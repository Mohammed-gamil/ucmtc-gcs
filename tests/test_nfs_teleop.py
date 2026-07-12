"""Unit tests for the PS5 NFS-style Rover Teleop system."""

import json
import os
import sys
import unittest
from unittest.mock import MagicMock

# Add workspace to path
WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WORKSPACE_DIR not in sys.path:
    sys.path.insert(0, WORKSPACE_DIR)
# Add rover_ws src to path
ROVER_SRC_DIR = os.path.join(WORKSPACE_DIR, "rover_ws", "src", "rover_core")
if ROVER_SRC_DIR not in sys.path:
    sys.path.insert(0, ROVER_SRC_DIR)

from rover_core.ps5_nfs_teleop import Ps5NfsTeleop
from rover_core.cmd_vel_to_wheels import CmdVelToWheels
from rover_core.wheel_cmds_serial_bridge import WheelCmdsSerialBridge


class TestNfsTeleopPhysics(unittest.TestCase):
    """Test NFS Teleop node physics simulation."""

    def test_arming_disarming(self):
        node = Ps5NfsTeleop()
        self.assertFalse(node.armed)

        # Trigger arming button (Options = index 9)
        joy_msg = MagicMock()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12
        joy_msg.buttons[9] = 1 # Arm
        node.joy_callback(joy_msg)
        self.assertTrue(node.armed)

        # Trigger disarming button (Share = index 8)
        joy_msg.buttons[9] = 0
        joy_msg.buttons[8] = 1 # Disarm
        node.joy_callback(joy_msg)
        self.assertFalse(node.armed)

    def test_trigger_normalization(self):
        node = Ps5NfsTeleop()
        # Trigger at rest (+1.0)
        norm_rest = node.normalize_trigger(1.0, 1.0, -1.0)
        self.assertEqual(norm_rest, 0.0)

        # Trigger pressed halfway (0.0)
        norm_half = node.normalize_trigger(0.0, 1.0, -1.0)
        self.assertEqual(norm_half, 0.5)

        # Trigger fully pressed (-1.0)
        norm_full = node.normalize_trigger(-1.0, 1.0, -1.0)
        self.assertEqual(norm_full, 1.0)

    def test_acceleration_and_coasting(self):
        node = Ps5NfsTeleop()
        node.armed = True
        node.current_speed = 0.0
        
        # Simulate pressing gas (R2 = raw_throttle = -1.0 -> normalized = 1.0)
        node.raw_throttle = -1.0
        node.raw_brake = 1.0 # at rest
        node.last_joy_stamp = node.get_clock().now().nanoseconds / 1e9
        
        # Step update loop
        node.update_loop()
        dt = 1.0 / node.control_rate_hz
        expected_speed = node.accel_rate * dt # curve is 1.0^2 = 1.0
        self.assertAlmostEqual(node.current_speed, expected_speed, places=4)

        # Coasting: set gas at rest (raw_throttle = 1.0)
        node.raw_throttle = 1.0
        node.current_speed = 1.0
        node.update_loop()
        expected_coast = 1.0 - node.coast_decel * dt
        self.assertAlmostEqual(node.current_speed, expected_coast, places=4)

    def test_braking_and_reversing(self):
        node = Ps5NfsTeleop()
        node.armed = True
        node.last_joy_stamp = node.get_clock().now().nanoseconds / 1e9

        # Case A: Moving forward fast, apply brake -> should slow down
        node.current_speed = 1.5
        node.raw_throttle = 1.0
        node.raw_brake = -1.0 # full brake (1.0 norm)
        node.update_loop()
        dt = 1.0 / node.control_rate_hz
        expected_speed = 1.5 - node.decel_rate * dt
        self.assertAlmostEqual(node.current_speed, expected_speed, places=4)

        # Case B: Stopped, apply brake -> should reverse
        node.current_speed = 0.0
        node.update_loop()
        self.assertLess(node.current_speed, 0.0)

    def test_nitro_boost(self):
        node = Ps5NfsTeleop()
        node.armed = True
        node.current_speed = 1.0
        node.boost_meter = 1.0
        node.boost_requested = True
        node.raw_throttle = -1.0 # gas on
        node.raw_brake = 1.0
        node.last_joy_stamp = node.get_clock().now().nanoseconds / 1e9

        node.update_loop()
        dt = 1.0 / node.control_rate_hz
        # Boost meter should deplete
        self.assertLess(node.boost_meter, 1.0)
        # Top speed threshold increases
        expected_top = node.forward_max * node.boost_multiplier
        self.assertEqual(node.forward_max * node.boost_multiplier, expected_top)

    def test_drift_behavior(self):
        node = Ps5NfsTeleop()
        node.armed = True
        node.current_speed = 2.0
        node.raw_steer = 1.0 # full steer
        node.drift_requested = True
        node.last_joy_stamp = node.get_clock().now().nanoseconds / 1e9

        node.update_loop()
        # Speed should decay by both coasting decel and traction loss
        dt = 1.0 / node.control_rate_hz
        expected = (2.0 - node.coast_decel * dt) * node.drift_traction_loss
        self.assertAlmostEqual(node.current_speed, expected, places=4)

    def test_watchdog_trigger(self):
        node = Ps5NfsTeleop()
        node.armed = True
        node.current_speed = 1.0
        # Joy time is way in the past
        node.last_joy_stamp = 0.0

        node.update_loop()
        # Watchdog must force full brake (speed decreases by decel_rate)
        dt = 1.0 / node.control_rate_hz
        self.assertAlmostEqual(node.current_speed, 1.0 - node.decel_rate * dt, places=4)


class TestNfsWheelMixer(unittest.TestCase):
    """Test CmdVelToWheels mixing."""

    def test_symmetric_skid_steer(self):
        node = CmdVelToWheels()
        node.wheel_pub = MagicMock()

        # Mock incoming cmd_vel (1.0 m/s straight forward, 0.0 turn)
        twist_msg = MagicMock()
        twist_msg.linear.x = 1.0
        twist_msg.angular.z = 0.0
        node.cmd_vel_callback(twist_msg)

        # Retrieve published value
        published_msg = node.wheel_pub.publish.call_args[0][0]
        # For straight forward, all wheels should have equal speed (1.0 / max_speed = 1.0 / 2.5 = 0.4)
        self.assertAlmostEqual(published_msg.data[0], 0.4, places=4)
        self.assertAlmostEqual(published_msg.data[1], 0.4, places=4)
        self.assertAlmostEqual(published_msg.data[2], 0.4, places=4)
        self.assertAlmostEqual(published_msg.data[3], 0.4, places=4)


class TestNfsSerialBridge(unittest.TestCase):
    """Test WheelCmdsSerialBridge framing and heartbeats."""

    def test_packet_framing(self):
        bridge = WheelCmdsSerialBridge()
        bridge._serial_conn = MagicMock()
        bridge._serial_conn.is_open = True
        bridge._armed = True
        bridge._safety_blocked = False

        # Set mock cmd_vel commands
        bridge._last_linear_x = 1.0
        bridge._last_angular_z = -0.5
        bridge._drift_active = True
        
        # Trigger control loop (which queues serial write)
        bridge.control_loop()

        # Verify command queued in queue:
        # linear_x_int = 1000, angular_z_int = -500, drift_active_int = 1
        # Checksum = (1000 + -500 + 1) & 0xFF = 501 & 0xFF = 245
        expected_bytes = b"<1000,-500,1,245>\n"
        
        # Fetch from queue
        packet = bridge._serial_queue.get(timeout=0.1)
        self.assertEqual(packet, expected_bytes)
