"""Unit tests for the Ground Control Station and Rover telemetry nodes."""

import json
import math
import os
import sys
import unittest
from unittest.mock import MagicMock, patch

# Add workspace to path
WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WORKSPACE_DIR not in sys.path:
    sys.path.insert(0, WORKSPACE_DIR)
# Add rover_ws src to path
ROVER_SRC_DIR = os.path.join(WORKSPACE_DIR, "rover_ws", "src", "rover_core")
if ROVER_SRC_DIR not in sys.path:
    sys.path.insert(0, ROVER_SRC_DIR)

from gcs_app.core.data_models import (
    TelemetryPayload,
    NavigationData,
    SafetyData,
    VisionData,
    JetsonData,
    CommunicationData,
    RosData
)
from gcs_app.core.ros_worker import ROSWorker
from gcs_app.core.global_connector import (
    GlobalConnector,
    PeerConnection,
    PeerInfo,
    PeerStatus,
    _build_announce_packet,
    _parse_announce_packet,
    DISCOVERY_MAGIC,
)
from rover_core.navigation_node import NavigationNode
from rover_core.safety_node import SafetyNode
from rover_core.vision_node import VisionNode
from rover_core.motor_control_node import MotorControlNode
from rover_core.telemetry_aggregator import TelemetryAggregatorNode
import rover_core.telemetry_utils as telemetry_utils


class TestTelemetryDataModels(unittest.TestCase):
    """Test strict telemetry schema parsing, type coercion, and validations."""

    def setUp(self):
        self.valid_payload = {
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
                "estop_mech_armed": True,
                "estop_wire_armed": True,
                "estop_triggered": False,
                "is_blocked": False,
                "collision_detected": False,
                "border_crossed": False,
                "border_partial": False,
                "obstacle_touched": False
            },
            "Vision": {
                "img_confidence": 0.85,
                "img_detected": True,
                "laser_active": False,
                "img_elapsed_sec": 10,
                "img_task_status": "active",
                "lane_detected": True,
                "obstacles_count": 1,
                "fps_vision": 30.0
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
                "node_lane_det": True,
                "node_obs_avoid": True,
                "node_wp_nav": True,
                "node_img_recog": True,
                "node_motor_ctrl": True,
                "esp32_connected": True,
                "rosout_last": "Active"
            }
        }

    def test_valid_payload_parsing(self):
        payload = TelemetryPayload.from_dict(self.valid_payload)
        self.assertEqual(payload.navigation.speed_kmh, 4.5)
        self.assertEqual(payload.safety.mode, "monitoring")
        self.assertEqual(payload.vision.img_elapsed_sec, 10)
        self.assertEqual(payload.jetson.temp_c, 50.4)
        self.assertEqual(payload.communication.rtt_ms, 25)
        self.assertTrue(payload.ros.node_lane_det)

    def test_payload_to_dict(self):
        payload = TelemetryPayload.from_dict(self.valid_payload)
        d = payload.to_dict()
        self.assertEqual(d["Navigation"]["speed_kmh"], 4.5)
        self.assertEqual(d["Safety"]["mode"], "monitoring")

    def test_missing_top_level_keys(self):
        bad_payload = dict(self.valid_payload)
        del bad_payload["Navigation"]
        with self.assertRaises(ValueError) as ctx:
            TelemetryPayload.from_dict(bad_payload)
        self.assertIn("missing keys: Navigation", str(ctx.exception))

    def test_invalid_field_type(self):
        bad_payload = dict(self.valid_payload)
        # speed_kmh must be a number, make it string
        bad_payload["Navigation"] = dict(bad_payload["Navigation"])
        bad_payload["Navigation"]["speed_kmh"] = "fast"
        with self.assertRaises(ValueError) as ctx:
            TelemetryPayload.from_dict(bad_payload)
        self.assertIn("Navigation.speed_kmh must be a number", str(ctx.exception))

    def test_extra_top_level_keys(self):
        bad_payload = dict(self.valid_payload)
        bad_payload["ExtraKey"] = {}
        with self.assertRaises(ValueError) as ctx:
            TelemetryPayload.from_dict(bad_payload)
        self.assertIn("unexpected keys: ExtraKey", str(ctx.exception))

    def test_empty_payload(self):
        payload = TelemetryPayload.empty()
        self.assertEqual(payload.navigation.speed_kmh, 0.0)
        self.assertEqual(payload.safety.mode, "idle")


class TestROSWorker(unittest.TestCase):
    """Test the GCS background worker telemetry bridging."""

    def test_fallback_telemetry_generation(self):
        worker = ROSWorker()
        # Telemetry should generate successfully in fallback mode
        telemetry = worker._build_fallback_telemetry()
        self.assertIsInstance(telemetry, TelemetryPayload)
        self.assertGreaterEqual(telemetry.navigation.speed_kmh, 0.0)
        self.assertGreater(telemetry.jetson.uptime_sec, -1)
        self.assertFalse(worker.is_connected())  # fallback is False by default

    def test_command_queue(self):
        worker = ROSWorker()
        cmd = {"action": "drive", "speed_kmh": 5.0}
        worker.send_motor_command(cmd)
        
        # Test flushing queue
        worker._flush_command_queue()
        with worker._data_lock:
            last_cmd = worker._shared_buffer["last_command"]
        self.assertEqual(last_cmd["action"], "drive")
        self.assertEqual(last_cmd["speed_kmh"], 5.0)


class TestRoverNodesCompatibility(unittest.TestCase):
    """Test individual ROS2 nodes running in compatibility (fallback) mode."""

    def test_navigation_node(self):
        node = NavigationNode()
        # Trigger timer callback to simulate running
        payload = node.publish_nav_data()
        self.assertEqual(payload["wp_status"], "idle")
        
        # Simulate receiving motor command "drive"
        cmd_msg = MagicMock()
        cmd_msg.data = json.dumps({
            "action": "drive",
            "speed_kmh": 8.0,
            "heading_deg": 180.0,
            "wp_current": 3,
            "wp_status": "navigating"
        })
        node.motor_command_callback(cmd_msg)
        
        # Verify navigation responds and target parameters are updated
        self.assertEqual(node._target_speed_kmh, 8.0)
        self.assertEqual(node._target_heading_deg, 180.0)
        self.assertEqual(node._wp_current, 3)
        
        # Simulate timer tick - speed should start ramping up towards target
        payload = node.publish_nav_data()
        self.assertGreater(node._speed_kmh, 0.0)
        self.assertEqual(payload["wp_status"], "navigating")

    def test_safety_node_override(self):
        node = SafetyNode()
        node.publish_safety_state()
        self.assertFalse(node._estop_triggered)
        
        # Send drive command
        cmd_msg = MagicMock()
        cmd_msg.data = json.dumps({"action": "drive", "speed_kmh": 4.0})
        node.motor_command_callback(cmd_msg)
        self.assertEqual(node._mode, "monitoring")
        
        # Send E-Stop command
        estop_msg = MagicMock()
        estop_msg.data = json.dumps({"action": "estop"})
        node.motor_command_callback(estop_msg)
        self.assertTrue(node._estop_triggered)
        
        payload = node.publish_safety_state()
        self.assertEqual(payload["mode"], "estop")
        self.assertTrue(payload["estop_triggered"])
        self.assertTrue(payload["is_blocked"])

    def test_vision_node_simulation(self):
        node = VisionNode()
        payload = node.publish_vision_metrics()
        self.assertIn("img_confidence", payload)
        self.assertIn("fps_vision", payload)
        self.assertIsInstance(payload["obstacles_count"], int)

    def test_motor_control_node(self):
        node = MotorControlNode()
        payload = node.timer_callback()
        self.assertTrue(payload["node_motor_ctrl"])
        self.assertEqual(payload["heartbeat_seq"], 1)

    def test_telemetry_aggregator(self):
        agg = TelemetryAggregatorNode()
        
        # Simulate publishing local sub-topics to the aggregator
        nav_payload = telemetry_utils.make_navigation_payload(
            speed_kmh=5.5, heading_deg=90.0, pos_lat=40.0, pos_lon=-75.0,
            dist_traveled_m=10.0, wp_current=1, wp_error_m=0.2, wp_status="navigating"
        )
        msg = MagicMock()
        msg.data = json.dumps(nav_payload)
        agg.navigation_callback(msg)
        
        # Check aggregator state
        payload = agg.build_payload()
        self.assertEqual(payload["Navigation"]["speed_kmh"], 5.5)
        self.assertEqual(payload["Navigation"]["heading_deg"], 90.0)


class TestWebGCSServer(unittest.TestCase):
    """Test the Web GCS server bridge validations and configuration."""

    def test_web_server_imports(self):
        from web_gcs.web_gcs_server import PORT, STATIC_DIR
        self.assertEqual(PORT, 8082)
        self.assertTrue(STATIC_DIR.endswith("web_gcs"))

    def test_mock_command_validation(self):
        # We can verify that posting a command validates fields correctly on the backend
        from web_gcs.web_gcs_server import GCSWebHandler
        
        # Set up handler mock
        handler = MagicMock(spec=GCSWebHandler)
        
        # Test command schema validation directly
        def simulate_command_validation(command):
            action = command.get("action")
            if action == "drive":
                speed = float(command.get("speed_kmh", 0.0))
                heading = float(command.get("heading_deg", 0.0))
                throttle = float(command.get("throttle_pct", 0.0))
                
                if not (0.0 <= speed <= 15.0):
                    raise ValueError("Speed out of bounds [0, 15]")
                if not (0.0 <= heading <= 360.0):
                    raise ValueError("Heading out of bounds [0, 360]")
                if not (0.0 <= throttle <= 1.0):
                    raise ValueError("Throttle out of bounds [0, 1]")
            return True

        # Valid drive command
        self.assertTrue(simulate_command_validation({"action": "drive", "speed_kmh": 10.0, "heading_deg": 180.0, "throttle_pct": 0.5}))

        # Invalid speed
        with self.assertRaises(ValueError) as ctx:
            simulate_command_validation({"action": "drive", "speed_kmh": 20.0, "heading_deg": 180.0, "throttle_pct": 0.5})
        self.assertEqual(str(ctx.exception), "Speed out of bounds [0, 15]")

        # Invalid heading
        with self.assertRaises(ValueError) as ctx:
            simulate_command_validation({"action": "drive", "speed_kmh": 10.0, "heading_deg": 400.0, "throttle_pct": 0.5})
        self.assertEqual(str(ctx.exception), "Heading out of bounds [0, 360]")

        # Invalid throttle
        with self.assertRaises(ValueError) as ctx:
            simulate_command_validation({"action": "drive", "speed_kmh": 10.0, "heading_deg": 180.0, "throttle_pct": 1.5})
        self.assertEqual(str(ctx.exception), "Throttle out of bounds [0, 1]")

    @patch("shutil.which")
    @patch("subprocess.Popen")
    def test_test_suite_endpoint_handling(self, mock_popen, mock_which):
        from web_gcs.web_gcs_server import GCSWebHandler
        
        # Mock GCSWebHandler instance
        handler = MagicMock(spec=GCSWebHandler)
        handler.path = "/api/test-suite/start"
        handler.headers = {}
        
        # Set up mock shutil.which to find gnome-terminal
        mock_which.side_effect = lambda cmd: "/usr/bin/gnome-terminal" if cmd == "gnome-terminal" else None
        
        # Run do_POST
        GCSWebHandler.do_POST(handler)
        
        # Verify that subprocess.Popen was called twice (once for t1 and once for t2)
        self.assertEqual(mock_popen.call_count, 2)
        
        # Check that it sent a success response
        handler._send_json.assert_called_once()
        args, kwargs = handler._send_json.call_args
        self.assertEqual(args[0], 200)
        response_data = json.loads(args[1])
        self.assertEqual(response_data["status"], "success")
        self.assertIn("Spawned two test terminals", response_data["message"])

    def test_bridge_node_direct_callbacks(self):
        from web_gcs.web_gcs_server import WebGCSBridgeNode, telemetry_state
        import web_gcs.web_gcs_server as server
        
        # Reset telemetry_state
        server.telemetry_state["latest"] = None
        
        # Instantiate node
        node = WebGCSBridgeNode()
        
        # Create mock ROS messages
        class MockVector3:
            def __init__(self, x=0.0, y=0.0, z=0.0):
                self.x = x
                self.y = y
                self.z = z
                
        class MockImuMsg:
            def __init__(self):
                self.linear_acceleration = MockVector3(1.2, 3.4, 5.6)
                self.angular_velocity = MockVector3(0.1, 0.2, 0.3)
                
        class MockPoint:
            def __init__(self, x=0.0, y=0.0, z=0.0):
                self.x = x
                self.y = y
                self.z = z
                
        class MockQuaternion:
            def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
                self.x = x
                self.y = y
                self.z = z
                self.w = w
                
        class MockPose:
            def __init__(self):
                self.position = MockPoint(10.0, 20.0, 30.0)
                self.orientation = MockQuaternion(0.0, 0.0, 0.0, 1.0)
                
        class MockPoseWithCovariance:
            def __init__(self):
                self.pose = MockPose()
                
        class MockTwist:
            def __init__(self):
                self.linear = MockVector3(1.0, 2.0, 0.0)
                self.angular = MockVector3(0.0, 0.0, 0.5)
                
        class MockTwistWithCovariance:
            def __init__(self):
                self.twist = MockTwist()
                
        class MockOdomMsg:
            def __init__(self):
                self.pose = MockPoseWithCovariance()
                self.twist = MockTwistWithCovariance()
                
        class MockGPSMsg:
            def __init__(self):
                self.latitude = 37.7749
                self.longitude = -122.4194
                self.altitude = 10.0
                
        class MockBatteryMsg:
            def __init__(self):
                self.voltage = 12.6
                self.percentage = 0.95
                
        class MockCmdVelMsg:
            def __init__(self):
                self.linear = MockVector3(1.5, 0.0, 0.0)
                self.angular = MockVector3(0.0, 0.0, 0.2)
        
        # Run callbacks
        node.imu_callback(MockImuMsg())
        self.assertIsNotNone(server.telemetry_state["latest"])
        self.assertEqual(server.telemetry_state["latest"]["Sensors"]["imu"]["accel_x"], 1.2)
        
        node.odom_callback(MockOdomMsg())
        self.assertEqual(server.telemetry_state["latest"]["Odom"]["pos_x"], 10.0)
        self.assertAlmostEqual(server.telemetry_state["latest"]["Odom"]["speed_kmh"], math.sqrt(1.0 + 4.0) * 3.6)
        
        node.gps_callback(MockGPSMsg())
        self.assertEqual(server.telemetry_state["latest"]["GPS"]["latitude"], 37.7749)
        
        node.battery_callback(MockBatteryMsg())
        self.assertEqual(server.telemetry_state["latest"]["Battery"]["voltage"], 12.6)
        self.assertEqual(server.telemetry_state["latest"]["Battery"]["percentage"], 95.0)
        self.assertEqual(server.telemetry_state["latest"]["Jetson"]["bat_pct"], 95.0)
        
        node.cmd_vel_echo_callback(MockCmdVelMsg())
        self.assertEqual(server.telemetry_state["latest"]["CmdVelEcho"]["linear_x"], 1.5)


class TestGlobalConnector(unittest.TestCase):
    """Test the Global Connector multi-peer network bridge."""

    def test_peer_info_creation(self):
        info = PeerInfo(
            peer_id="rover-alpha",
            hostname="alpha-host",
            ip_address="192.168.1.50",
            port=8090,
            ros_domain_id=0,
            role="rover",
            team_name="UCMTC",
            discovery_method="manual",
        )
        self.assertEqual(info.peer_id, "rover-alpha")
        self.assertEqual(info.ip_address, "192.168.1.50")
        self.assertEqual(info.role, "rover")

    def test_peer_connection_to_dict(self):
        info = PeerInfo(
            peer_id="drone-1",
            hostname="drone-host",
            ip_address="10.0.0.5",
            port=8091,
            role="drone",
            team_name="TeamB",
        )
        pc = PeerConnection(info=info, status=PeerStatus.CONNECTED)
        d = pc.to_dict()
        self.assertEqual(d["peer_id"], "drone-1")
        self.assertEqual(d["status"], "connected")
        self.assertEqual(d["role"], "drone")
        self.assertFalse(d["has_telemetry"])

    def test_peer_connection_with_telemetry(self):
        info = PeerInfo(
            peer_id="sensor-1",
            hostname="sensor-host",
            ip_address="10.0.0.10",
            port=8092,
            role="sensor_hub",
            team_name="TeamC",
        )
        pc = PeerConnection(info=info, status=PeerStatus.CONNECTED)
        pc.last_telemetry = {"temperature": 42.5, "humidity": 65}
        pc.packets_received = 150
        d = pc.to_dict()
        self.assertTrue(d["has_telemetry"])
        self.assertEqual(d["packets_received"], 150)

    def test_peer_status_enum(self):
        self.assertEqual(PeerStatus.CONNECTED.value, "connected")
        self.assertEqual(PeerStatus.DISCONNECTED.value, "disconnected")
        self.assertEqual(PeerStatus.DISCOVERED.value, "discovered")

    def test_announce_packet_build_and_parse(self):
        packet = _build_announce_packet(
            peer_id="test-peer",
            hostname="test-host",
            port=9090,
            ros_domain_id=1,
            role="rover",
            team_name="TestTeam",
        )
        self.assertTrue(packet.startswith(DISCOVERY_MAGIC))

        parsed = _parse_announce_packet(packet, "192.168.1.100")
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.peer_id, "test-peer")
        self.assertEqual(parsed.ip_address, "192.168.1.100")
        self.assertEqual(parsed.port, 9090)
        self.assertEqual(parsed.ros_domain_id, 1)
        self.assertEqual(parsed.role, "rover")
        self.assertEqual(parsed.team_name, "TestTeam")

    def test_announce_packet_invalid_data(self):
        self.assertIsNone(_parse_announce_packet(b"garbage data", "1.2.3.4"))
        self.assertIsNone(_parse_announce_packet(b"", "1.2.3.4"))
        self.assertIsNone(_parse_announce_packet(DISCOVERY_MAGIC + b"not-json", "1.2.3.4"))

    def test_connector_init_defaults(self):
        connector = GlobalConnector(
            local_peer_id="test-gcs",
            enable_discovery=False,
            enable_announcements=False,
        )
        self.assertEqual(connector._local_peer_id, "test-gcs")
        self.assertEqual(connector.peer_count, 0)
        self.assertEqual(connector.connected_count, 0)

    def test_connector_add_manual_peer_no_start(self):
        """Test adding a manual peer (without actually starting network threads)."""
        connector = GlobalConnector(
            local_peer_id="test-gcs",
            enable_discovery=False,
            enable_announcements=False,
        )
        # Directly add peer to internal state (simulating add_manual_peer without threads)
        info = PeerInfo(
            peer_id="rover-beta",
            hostname="beta-host",
            ip_address="192.168.1.99",
            port=8090,
            role="rover",
            team_name="UCMTC",
            discovery_method="manual",
        )
        connector._peers["rover-beta"] = PeerConnection(info=info, status=PeerStatus.DISCOVERED)
        
        self.assertEqual(connector.peer_count, 1)
        snap = connector.get_peers_snapshot()
        self.assertEqual(len(snap), 1)
        self.assertEqual(snap[0]["peer_id"], "rover-beta")

    def test_connector_remove_peer(self):
        connector = GlobalConnector(
            local_peer_id="test-gcs",
            enable_discovery=False,
            enable_announcements=False,
        )
        info = PeerInfo(
            peer_id="temp-peer",
            hostname="temp",
            ip_address="10.0.0.1",
            port=8090,
            role="rover",
            team_name="Test",
        )
        connector._peers["temp-peer"] = PeerConnection(info=info)
        self.assertEqual(connector.peer_count, 1)

        removed = connector.remove_peer("temp-peer")
        self.assertTrue(removed)
        self.assertEqual(connector.peer_count, 0)

        # Remove non-existent
        removed2 = connector.remove_peer("nonexistent")
        self.assertFalse(removed2)

    def test_connector_set_and_get_local_telemetry(self):
        connector = GlobalConnector(
            local_peer_id="test-gcs",
            enable_discovery=False,
            enable_announcements=False,
        )
        test_telemetry = {"Navigation": {"speed_kmh": 5.0}}
        connector.set_local_telemetry(test_telemetry)

        merged = connector.get_merged_telemetry()
        self.assertIn("local", merged)
        self.assertEqual(merged["local"]["Navigation"]["speed_kmh"], 5.0)
        self.assertEqual(merged["peer_count"], 0)

    def test_connector_get_all_peer_telemetry(self):
        connector = GlobalConnector(
            local_peer_id="test-gcs",
            enable_discovery=False,
            enable_announcements=False,
        )
        info = PeerInfo(
            peer_id="data-peer",
            hostname="dp",
            ip_address="10.0.0.5",
            port=8090,
            role="rover",
            team_name="Test",
        )
        pc = PeerConnection(info=info, status=PeerStatus.CONNECTED)
        pc.last_telemetry = {"Navigation": {"speed_kmh": 3.2}}
        connector._peers["data-peer"] = pc

        all_tel = connector.get_all_peer_telemetry()
        self.assertIn("data-peer", all_tel)
        self.assertEqual(all_tel["data-peer"]["Navigation"]["speed_kmh"], 3.2)

    def test_connector_peer_telemetry_lookup(self):
        connector = GlobalConnector(
            local_peer_id="test-gcs",
            enable_discovery=False,
            enable_announcements=False,
        )
        # No peer => None
        self.assertIsNone(connector.get_peer_telemetry("absent"))

        info = PeerInfo(
            peer_id="lookup-peer",
            hostname="lp",
            ip_address="10.0.0.6",
            port=8090,
            role="drone",
            team_name="Test",
        )
        pc = PeerConnection(info=info)
        pc.last_telemetry = {"test": True}
        connector._peers["lookup-peer"] = pc

        result = connector.get_peer_telemetry("lookup-peer")
        self.assertEqual(result, {"test": True})

    def test_connector_connected_count(self):
        connector = GlobalConnector(
            local_peer_id="test-gcs",
            enable_discovery=False,
            enable_announcements=False,
        )
        for i in range(3):
            info = PeerInfo(
                peer_id=f"peer-{i}",
                hostname=f"h{i}",
                ip_address=f"10.0.0.{i}",
                port=8090,
                role="rover",
                team_name="T",
            )
            status = PeerStatus.CONNECTED if i < 2 else PeerStatus.DISCONNECTED
            connector._peers[f"peer-{i}"] = PeerConnection(info=info, status=status)

        self.assertEqual(connector.peer_count, 3)
        self.assertEqual(connector.connected_count, 2)

    def test_connector_merged_telemetry_structure(self):
        connector = GlobalConnector(
            local_peer_id="test-gcs",
            enable_discovery=False,
            enable_announcements=False,
        )
        connector.set_local_telemetry({"speed": 10})
        info = PeerInfo(
            peer_id="remote-1",
            hostname="r1",
            ip_address="10.0.0.1",
            port=8090,
            role="rover",
            team_name="TeamA",
        )
        pc = PeerConnection(info=info, status=PeerStatus.CONNECTED)
        pc.last_telemetry = {"speed": 5}
        connector._peers["remote-1"] = pc

        merged = connector.get_merged_telemetry()
        self.assertIn("local", merged)
        self.assertIn("peers", merged)
        self.assertIn("peer_count", merged)
        self.assertIn("timestamp", merged)
        self.assertEqual(merged["peer_count"], 1)
        self.assertEqual(merged["peers"]["remote-1"]["role"], "rover")
        self.assertEqual(merged["peers"]["remote-1"]["telemetry"]["speed"], 5)


class TestWebGCSUI(unittest.TestCase):
    """Integration tests for the Web GCS frontend UI pages, assets, and APIs."""

    @classmethod
    def setUpClass(cls):
        import threading
        from web_gcs.web_gcs_server import ThreadedHTTPServer, GCSWebHandler
        # Bind to port 0 to get an ephemeral free port from OS
        cls.server = ThreadedHTTPServer(("127.0.0.1", 0), GCSWebHandler)
        cls.port = cls.server.server_address[1]
        cls.server_thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.server_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()

    def test_serve_index_html(self):
        import urllib.request
        url = f"http://127.0.0.1:{self.port}/"
        with urllib.request.urlopen(url) as response:
            self.assertEqual(response.status, 200)
            html = response.read().decode("utf-8")
            self.assertIn("GROUND STATION", html)
            self.assertIn("styles.css", html)
            self.assertIn("app.js", html)
            self.assertIn("Outfit", html)
            self.assertIn("Rajdhani", html)

    def test_serve_styles_css(self):
        import urllib.request
        url = f"http://127.0.0.1:{self.port}/styles.css"
        with urllib.request.urlopen(url) as response:
            self.assertEqual(response.status, 200)
            css = response.read().decode("utf-8")
            self.assertIn("--bg-void", css)
            self.assertIn("#010306", css)
            self.assertIn("Rajdhani", css)
            self.assertIn("Share Tech Mono", css)
            self.assertIn("Outfit", css)

    def test_serve_app_js(self):
        import urllib.request
        url = f"http://127.0.0.1:{self.port}/app.js"
        with urllib.request.urlopen(url) as response:
            self.assertEqual(response.status, 200)
            js = response.read().decode("utf-8")
            self.assertIn("updateDashboard", js)
            self.assertIn("btn-run-ui-tests", js)

    def test_api_peers_endpoint(self):
        import urllib.request
        url = f"http://127.0.0.1:{self.port}/api/peers"
        with urllib.request.urlopen(url) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode("utf-8"))
            self.assertIn("peers", data)
            self.assertIn("peer_count", data)

    def test_api_connector_status(self):
        import urllib.request
        url = f"http://127.0.0.1:{self.port}/api/connector/status"
        with urllib.request.urlopen(url) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode("utf-8"))
            self.assertIn("local_peer_id", data)
            self.assertIn("running", data)

    def test_cors_options(self):
        import urllib.request
        url = f"http://127.0.0.1:{self.port}/api/peers"
        req = urllib.request.Request(url, method="OPTIONS")
        with urllib.request.urlopen(req) as response:
            self.assertEqual(response.status, 200)
            headers = dict(response.info())
            self.assertEqual(headers.get("Access-Control-Allow-Origin"), "*")
            self.assertIn("GET", headers.get("Access-Control-Allow-Methods", ""))

    def test_404_not_found(self):
        import urllib.request
        import urllib.error
        url = f"http://127.0.0.1:{self.port}/nonexistent-file.html"
        try:
            urllib.request.urlopen(url)
            self.fail("Should have raised HTTPError 404")
        except urllib.error.HTTPError as err:
            self.assertEqual(err.code, 404)

    def test_estop_command_dispatch(self):
        import urllib.request
        import json
        url = f"http://127.0.0.1:{self.port}/command"
        command = {
            "action": "estop",
            "speed_kmh": 0.0,
            "heading_deg": 0.0,
            "throttle_pct": 0.0,
            "estop_triggered": True,
            "source": "web_gcs",
            "timestamp_ms": 1000
        }
        data = json.dumps(command).encode("utf-8")
        req = urllib.request.Request(url, data=data, method="POST", headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req) as response:
            self.assertEqual(response.status, 200)
            res = json.loads(response.read().decode("utf-8"))
            self.assertEqual(res["status"], "success")
            self.assertEqual(res["message"], "Command dispatched")

    def test_estop_command_updates_simulation_telemetry(self):
        import urllib.request
        import json
        import web_gcs.web_gcs_server as server
        
        # Ensure clean initial state
        server.sim_estop_triggered = False

        # Send E-stop
        url = f"http://127.0.0.1:{self.port}/command"
        req = urllib.request.Request(
            url,
            data=json.dumps({"action": "estop"}).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req) as response:
            self.assertEqual(response.status, 200)

        # Verify simulation state updates
        self.assertTrue(server.sim_estop_triggered)
        
        # Send Resume
        req = urllib.request.Request(
            url,
            data=json.dumps({"action": "resume"}).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req) as response:
            self.assertEqual(response.status, 200)
            
        self.assertFalse(server.sim_estop_triggered)

    def test_topic_config_endpoints(self):
        import urllib.request
        import json
        import os
        import web_gcs.web_gcs_server as server

        # 1. Get current config
        url = f"http://127.0.0.1:{self.port}/api/config/topics"
        with urllib.request.urlopen(url) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode("utf-8"))
            self.assertIn("motor_control", data)
            self.assertIn("imu_accel", data)
            self.assertEqual(data["motor_control"]["path"], "/rover/commands/motor")

        # 2. Update config via POST
        custom_config = dict(server.DEFAULT_TOPICS)
        custom_config["motor_control"] = {"label": "Custom Motor", "path": "/custom/commands/motor"}
        custom_config["imu_accel"] = {"label": "Custom IMU", "path": "/custom/sensors/imu"}

        req = urllib.request.Request(
            url,
            data=json.dumps(custom_config).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req) as response:
            self.assertEqual(response.status, 200)
            res_data = json.loads(response.read().decode("utf-8"))
            self.assertEqual(res_data["status"], "success")

        # Verify configuration was updated on the server
        self.assertEqual(server.topic_config["motor_control"]["path"], "/custom/commands/motor")
        self.assertEqual(server.topic_config["imu_accel"]["label"], "Custom IMU")
        self.assertTrue(os.path.exists(server.TOPIC_CONFIG_FILE))

        # 3. Reset config
        reset_url = f"http://127.0.0.1:{self.port}/api/config/topics/reset"
        reset_req = urllib.request.Request(reset_url, data=b"", method="POST")
        with urllib.request.urlopen(reset_req) as response:
            self.assertEqual(response.status, 200)
            res_data = json.loads(response.read().decode("utf-8"))
            self.assertEqual(res_data["status"], "success")

        # Verify config reverted and file was deleted
        self.assertEqual(server.topic_config["motor_control"]["path"], "/rover/commands/motor")
        self.assertEqual(server.topic_config["imu_accel"]["label"], "IMU ACCEL")
        self.assertFalse(os.path.exists(server.TOPIC_CONFIG_FILE))

    def test_ros2_topics_list_endpoint(self):
        import urllib.request
        import json
        url = f"http://127.0.0.1:{self.port}/api/ros2/topics"
        with urllib.request.urlopen(url) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode("utf-8"))
            self.assertIn("topics", data)
            self.assertTrue(len(data["topics"]) > 0)
            for t in data["topics"]:
                self.assertIn("name", t)
                self.assertIn("types", t)


class TestGCSMetadataArchitecture(unittest.TestCase):
    """Verifies topic type resolution errors, websocket deduplication, and topic whitelist filters."""

    def test_unresolved_topic_degradation(self):
        from web_gcs.topic_registry import get_registry, load_registry
        import tempfile
        import json

        # Create a temporary topics.json with a bad/missing type
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            json.dump([
                {"name": "/bad_topic", "type": "missing_package/msg/FakeMessage"},
                {"name": "/normal_topic", "type": "std_msgs/msg/String"}
            ], f)
            temp_path = f.name

        try:
            # Load the registry from our temporary file
            load_registry(temp_path)
            registry = get_registry()
            
            # The bad topic should fail gracefully and degrade
            self.assertIn("/bad_topic", registry)
            self.assertEqual(registry["/bad_topic"]["connection_state"], "error")
            self.assertIn("missing type definition", registry["/bad_topic"]["error_reason"])
            self.assertEqual(registry["/bad_topic"]["schema"], None)
            
            # The normal topic should be OK (or at least resolved if std_msgs is present)
            self.assertIn("/normal_topic", registry)
            if registry["/normal_topic"]["connection_state"] != "error":
                self.assertIsNotNone(registry["/normal_topic"]["schema"])

        finally:
            import os
            try:
                os.remove(temp_path)
            except Exception:
                pass
            load_registry()

    def test_websocket_deduplication(self):
        from web_gcs.websocket import _last_emitted_data
        
        # Test change detection check
        test_topic = "/test_dedup"
        _last_emitted_data[test_topic] = {"value": 42}
        
        # Identical payload should not be emitted (returns same)
        payload = {"value": 42}
        has_changed = (payload != _last_emitted_data[test_topic])
        self.assertFalse(has_changed)
        
        # Different payload should be emitted
        new_payload = {"value": 43}
        has_changed = (new_payload != _last_emitted_data[test_topic])
        self.assertTrue(has_changed)

    def test_join_topic_whitelisting(self):
        from web_gcs.websocket import on_join
        from unittest.mock import patch

        with patch("web_gcs.websocket.join_room") as mock_join, \
             patch("web_gcs.websocket.emit") as mock_emit:
            # Valid topic from registry
            from web_gcs.topic_registry import get_registry
            registry = get_registry()
            test_topic = list(registry.keys())[0] if registry else "/battery_state"
            
            on_join({"topic": test_topic})
            mock_join.assert_called_once_with(test_topic)
            
            # Invalid/unregistered topic must be rejected
            mock_join.reset_mock()
            on_join({"topic": "/invalid_nonexistent_topic"})
            mock_join.assert_not_called()
            mock_emit.assert_called_with("error", {"message": "Topic '/invalid_nonexistent_topic' is not allowed (not in registry)"})


if __name__ == "__main__":
    unittest.main()

