#!/usr/bin/env python3
"""Node B: cmd_vel_to_wheels wheel mixing node.

Subscribes to:
- /cmd_vel (geometry_msgs/Twist)
- /rover/hud (std_msgs/String - JSON for drift status)
Publishes:
- /wheel_cmds (std_msgs/Float32MultiArray [FL, FR, RL, RR])
"""

from __future__ import annotations

import json
from typing import Any

from rover_core.ros_compat import Node, String, ensure_ros_initialized, run_node
from rover_core.telemetry_utils import COMMAND_QOS, SENSOR_QOS, RELIABLE_QOS

try:
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32MultiArray
except ImportError:
    Twist = None
    Float32MultiArray = None


class CmdVelToWheels(Node):
    """Subscribes to cmd_vel and outputs 4 wheel commands with drift mechanics."""

    def __init__(self):
        self._owns_ros_context = ensure_ros_initialized()
        super().__init__("cmd_vel_to_wheels")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("track_width", 0.5)            # meters
        self.declare_parameter("max_wheel_speed", 2.5)        # m/s (corresponds to full speed 1.0)
        self.declare_parameter("bias_fl", 1.0)
        self.declare_parameter("bias_fr", 1.0)
        self.declare_parameter("bias_rl", 1.0)
        self.declare_parameter("bias_rr", 1.0)
        self.declare_parameter("drift_traction_loss_inner", 0.6)
        self.declare_parameter("drift_traction_loss_outer", 1.0)

        # Get values
        self.track_width = self.get_parameter("track_width").value
        self.max_wheel_speed = self.get_parameter("max_wheel_speed").value
        self.bias_fl = self.get_parameter("bias_fl").value
        self.bias_fr = self.get_parameter("bias_fr").value
        self.bias_rl = self.get_parameter("bias_rl").value
        self.bias_rr = self.get_parameter("bias_rr").value
        self.drift_traction_loss_inner = self.get_parameter("drift_traction_loss_inner").value
        self.drift_traction_loss_outer = self.get_parameter("drift_traction_loss_outer").value

        # ── State ────────────────────────────────────────────────────────────
        self.drift_active = False

        # ── Pub/Sub ──────────────────────────────────────────────────────────
        self.cmd_vel_sub = self.create_subscription(
            Twist if Twist else Any,
            "/cmd_vel",
            self.cmd_vel_callback,
            COMMAND_QOS
        )

        self.hud_sub = self.create_subscription(
            String,
            "/rover/hud",
            self.hud_callback,
            RELIABLE_QOS
        )

        self.wheel_pub = self.create_publisher(
            Float32MultiArray if Float32MultiArray else Any,
            "/wheel_cmds",
            COMMAND_QOS
        )

        self.get_logger().info("CmdVelToWheels mixer node initialized.")

    def hud_callback(self, msg: Any) -> None:
        """Cache drift status from the teleop node HUD publication."""
        try:
            data = json.loads(msg.data)
            self.drift_active = bool(data.get("drift_active", False))
        except Exception:
            self.drift_active = False

    def cmd_vel_callback(self, msg: Any) -> None:
        """Perform 4-wheel mixing on receipt of cmd_vel."""
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        # 4-wheel configurations
        wheels = ["FL", "FR", "RL", "RR"]
        side = {"FL": -1.0, "RL": -1.0, "FR": 1.0, "RR": 1.0}
        bias = {
            "FL": self.bias_fl,
            "FR": self.bias_fr,
            "RL": self.bias_rl,
            "RR": self.bias_rr
        }

        # Calculate traction based on drift
        traction = {w: 1.0 for w in wheels}
        if self.drift_active:
            if angular_z > 0.05:
                # Left turn -> Left wheels (FL, RL) are inner
                traction["FL"] = self.drift_traction_loss_inner
                traction["RL"] = self.drift_traction_loss_inner
                traction["FR"] = self.drift_traction_loss_outer
                traction["RR"] = self.drift_traction_loss_outer
            elif angular_z < -0.05:
                # Right turn -> Right wheels (FR, RR) are inner
                traction["FR"] = self.drift_traction_loss_inner
                traction["RR"] = self.drift_traction_loss_inner
                traction["FL"] = self.drift_traction_loss_outer
                traction["RL"] = self.drift_traction_loss_outer

        # Apply mixing formula
        raw_speeds = {}
        for w in wheels:
            # V_wheel = (V_linear + side * V_angular * track_width / 2.0) * bias * traction
            raw_speeds[w] = (linear_x + side[w] * angular_z * (self.track_width / 2.0)) * bias[w] * traction[w]

        # Normalization and clipping prevention
        largest = max(max(abs(v) for v in raw_speeds.values()), self.max_wheel_speed)
        
        # Output values scaled between -1.0 and 1.0
        fl_val = raw_speeds["FL"] / largest
        fr_val = raw_speeds["FR"] / largest
        rl_val = raw_speeds["RL"] / largest
        rr_val = raw_speeds["RR"] / largest

        # Publish
        if Float32MultiArray is not None:
            out_msg = Float32MultiArray()
            out_msg.data = [fl_val, fr_val, rl_val, rr_val]
            self.wheel_pub.publish(out_msg)


def main(args=None):
    node = CmdVelToWheels()
    run_node(node, fallback_period=0.02)


if __name__ == "__main__":
    main()
