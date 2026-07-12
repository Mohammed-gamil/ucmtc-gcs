#!/usr/bin/env python3
"""Node A: PS5 NFS-style arcade racer teleop node.

Subscribes to /joy (sensor_msgs/Joy) and publishes:
- /cmd_vel (geometry_msgs/Twist)
- /rover/hud (std_msgs/String - JSON)
"""

from __future__ import annotations

import json
import time
from typing import Any

from rover_core.ros_compat import Node, String, ensure_ros_initialized, run_node
from rover_core.telemetry_utils import (
    COMMAND_QOS,
    RELIABLE_QOS,
    SENSOR_QOS,
    _STD_MSGS_AVAILABLE,
)

# Standard imports for messages if available
try:
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy
except ImportError:
    Twist = None
    Joy = None


class Ps5NfsTeleop(Node):
    """PS5 NFS-style vehicle dynamics simulator node."""

    def __init__(self):
        self._owns_ros_context = ensure_ros_initialized()
        super().__init__("ps5_nfs_teleop")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("axis_steer", 0)           # Left stick X
        self.declare_parameter("axis_throttle", 5)        # R2
        self.declare_parameter("axis_brake", 2)           # L2
        self.declare_parameter("button_boost", 5)         # R1
        self.declare_parameter("button_drift", 1)         # Circle / Cross
        self.declare_parameter("button_arm", 9)           # Options
        self.declare_parameter("button_disarm", 8)        # Share / Touchpad

        # Physics values
        self.declare_parameter("forward_max", 2.5)        # m/s
        self.declare_parameter("reverse_max", 1.2)        # m/s
        self.declare_parameter("accel_rate", 3.0)         # m/s^2
        self.declare_parameter("decel_rate", 6.0)         # m/s^2
        self.declare_parameter("reverse_accel_rate", 2.0) # m/s^2
        self.declare_parameter("coast_decel", 1.5)        # m/s^2
        self.declare_parameter("throttle_curve_exp", 2.0)
        self.declare_parameter("brake_curve_exp", 2.0)
        self.declare_parameter("reverse_engage_threshold", 0.05)

        # Steering values
        self.declare_parameter("steer_gain_low_speed", 1.8)  # rad/s
        self.declare_parameter("steer_gain_high_speed", 0.6) # rad/s
        self.declare_parameter("drift_steer_multiplier", 1.5)
        self.declare_parameter("drift_traction_loss", 0.98)  # per-tick multiplier

        # Nitro boost
        self.declare_parameter("boost_multiplier", 1.8)
        self.declare_parameter("boost_duration", 3.0)     # seconds
        self.declare_parameter("boost_regen_time", 8.0)    # seconds

        # Deadzones & watchdogs
        self.declare_parameter("deadzone_stick", 0.05)
        self.declare_parameter("deadzone_trigger", 0.05)
        self.declare_parameter("watchdog_timeout", 0.5)   # seconds
        self.declare_parameter("control_rate_hz", 50.0)

        # Trigger calibrations (standard triggers rest at 1.0, pressed at -1.0)
        self.declare_parameter("trigger_throttle_rest", 1.0)
        self.declare_parameter("trigger_throttle_pressed", -1.0)
        self.declare_parameter("trigger_brake_rest", 1.0)
        self.declare_parameter("trigger_brake_pressed", -1.0)

        # Get values
        self.axis_steer = self.get_parameter("axis_steer").value
        self.axis_throttle = self.get_parameter("axis_throttle").value
        self.axis_brake = self.get_parameter("axis_brake").value
        self.button_boost = self.get_parameter("button_boost").value
        self.button_drift = self.get_parameter("button_drift").value
        self.button_arm = self.get_parameter("button_arm").value
        self.button_disarm = self.get_parameter("button_disarm").value

        self.forward_max = self.get_parameter("forward_max").value
        self.reverse_max = self.get_parameter("reverse_max").value
        self.accel_rate = self.get_parameter("accel_rate").value
        self.decel_rate = self.get_parameter("decel_rate").value
        self.reverse_accel_rate = self.get_parameter("reverse_accel_rate").value
        self.coast_decel = self.get_parameter("coast_decel").value
        self.throttle_curve_exp = self.get_parameter("throttle_curve_exp").value
        self.brake_curve_exp = self.get_parameter("brake_curve_exp").value
        self.reverse_engage_threshold = self.get_parameter("reverse_engage_threshold").value

        self.steer_gain_low_speed = self.get_parameter("steer_gain_low_speed").value
        self.steer_gain_high_speed = self.get_parameter("steer_gain_high_speed").value
        self.drift_steer_multiplier = self.get_parameter("drift_steer_multiplier").value
        self.drift_traction_loss = self.get_parameter("drift_traction_loss").value

        self.boost_multiplier = self.get_parameter("boost_multiplier").value
        self.boost_duration = self.get_parameter("boost_duration").value
        self.boost_regen_time = self.get_parameter("boost_regen_time").value

        self.deadzone_stick = self.get_parameter("deadzone_stick").value
        self.deadzone_trigger = self.get_parameter("deadzone_trigger").value
        self.watchdog_timeout = self.get_parameter("watchdog_timeout").value
        self.control_rate_hz = self.get_parameter("control_rate_hz").value

        self.trigger_throttle_rest = self.get_parameter("trigger_throttle_rest").value
        self.trigger_throttle_pressed = self.get_parameter("trigger_throttle_pressed").value
        self.trigger_brake_rest = self.get_parameter("trigger_brake_rest").value
        self.trigger_brake_pressed = self.get_parameter("trigger_brake_pressed").value

        # ── Persistent State ─────────────────────────────────────────────────
        self.current_speed = 0.0        # m/s, signed
        self.boost_meter = 1.0          # 0..1
        self.drift_active = False
        self.armed = False

        # Input tracking
        self.raw_steer = 0.0
        self.raw_throttle = self.trigger_throttle_rest
        self.raw_brake = self.trigger_brake_rest
        self.boost_requested = False
        self.drift_requested = False
        self.last_joy_stamp = 0.0

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self.joy_sub = self.create_subscription(
            Joy if Joy else Any,
            "/joy",
            self.joy_callback,
            SENSOR_QOS
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist if Twist else Any,
            "/cmd_vel",
            COMMAND_QOS
        )

        self.hud_pub = self.create_publisher(
            String,
            "/rover/hud",
            RELIABLE_QOS
        )

        # ── Dynamic Update Timer ─────────────────────────────────────────────
        period_sec = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(period_sec, self.update_loop)

        self.get_logger().info(
            f"PS5 NFS Teleop initialized at {self.control_rate_hz} Hz. DISARMED."
        )

    def joy_callback(self, msg: Any) -> None:
        """Process incoming joy inputs."""
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_joy_stamp = now

        # Read stick & triggers safely
        if len(msg.axes) > max(self.axis_steer, self.axis_throttle, self.axis_brake):
            self.raw_steer = msg.axes[self.axis_steer]
            self.raw_throttle = msg.axes[self.axis_throttle]
            self.raw_brake = msg.axes[self.axis_brake]

        # Read buttons safely
        if len(msg.buttons) > max(self.button_boost, self.button_drift, self.button_arm, self.button_disarm):
            self.boost_requested = bool(msg.buttons[self.button_boost])
            self.drift_requested = bool(msg.buttons[self.button_drift])

            # Latching arm/disarm
            if msg.buttons[self.button_arm]:
                if not self.armed:
                    self.armed = True
                    self.get_logger().info("Vehicle NFS Teleop: ARMED")
            if msg.buttons[self.button_disarm]:
                if self.armed:
                    self.armed = False
                    self.get_logger().warning("Vehicle NFS Teleop: DISARMED")

    def normalize_trigger(self, raw: float, rest: float, pressed: float) -> float:
        """Convert raw trigger range to clean 0.0 - 1.0."""
        if abs(pressed - rest) < 1e-5:
            return 0.0
        val = (raw - rest) / (pressed - rest)
        return float(max(0.0, min(1.0, val)))

    def apply_deadzone(self, val: float, deadzone: float) -> float:
        """Ignore small stick noise and interpolate remaining range."""
        if abs(val) < deadzone:
            return 0.0
        sign_val = 1.0 if val > 0.0 else -1.0
        return float(sign_val * (abs(val) - deadzone) / (1.0 - deadzone))

    def sign(self, val: float) -> float:
        return 1.0 if val > 0.0 else (-1.0 if val < 0.0 else 0.0)

    def lerp(self, a: float, b: float, t: float) -> float:
        t = max(0.0, min(1.0, t))
        return a + t * (b - a)

    def throttle_curve(self, x: float) -> float:
        return abs(x) ** self.throttle_curve_exp

    def brake_curve(self, x: float) -> float:
        return abs(x) ** self.brake_curve_exp

    def update_loop(self) -> None:
        """Integrate vehicle state and publish commands."""
        dt = 1.0 / self.control_rate_hz
        now = self.get_clock().now().nanoseconds / 1e9

        # ── 1. Link Watchdog / Fail-safe ─────────────────────────────────────
        link_timeout_active = (now - self.last_joy_stamp) > self.watchdog_timeout

        if link_timeout_active:
            # Force zero throttle and full brake on link timeout
            throttle = 0.0
            brake = 1.0
            steer = 0.0
            boost_requested = False
            drift_requested = False
        else:
            throttle = self.normalize_trigger(self.raw_throttle, self.trigger_throttle_rest, self.trigger_throttle_pressed)
            brake = self.normalize_trigger(self.raw_brake, self.trigger_brake_rest, self.trigger_brake_pressed)
            steer = self.apply_deadzone(self.raw_steer, self.deadzone_stick)
            
            # Apply deadzones to triggers
            if throttle < self.deadzone_trigger:
                throttle = 0.0
            if brake < self.deadzone_trigger:
                brake = 0.0
                
            boost_requested = self.boost_requested
            drift_requested = self.drift_requested

        # ── 2. Disarmed State ────────────────────────────────────────────────
        if not self.armed:
            self.current_speed = 0.0
            self.boost_meter = min(1.0, self.boost_meter + dt / self.boost_regen_time)
            self.publish_twist(0.0, 0.0)
            self.publish_hud(0.0, self.boost_meter, False, "N")
            return

        # ── 3. Longitudinal Dynamics (Throttle/Brake/Coast) ──────────────────
        if self.current_speed >= 0.0:
            if brake > 0.0:
                if self.current_speed > self.reverse_engage_threshold:
                    self.current_speed -= self.decel_rate * self.brake_curve(brake) * dt
                else:
                    self.current_speed -= self.reverse_accel_rate * self.brake_curve(brake) * dt
            elif throttle > 0.0:
                self.current_speed += self.accel_rate * self.throttle_curve(throttle) * dt
            else:
                # Arcade coast decel
                self.current_speed -= self.sign(self.current_speed) * self.coast_decel * dt
                if abs(self.current_speed) < self.coast_decel * dt:
                    self.current_speed = 0.0
        else:
            # Reversing
            if throttle > 0.0:
                # Gas acts as brake for reverse
                self.current_speed += self.decel_rate * self.throttle_curve(throttle) * dt
                if self.current_speed > 0.0:
                    self.current_speed = 0.0
            elif brake > 0.0:
                self.current_speed -= self.reverse_accel_rate * self.brake_curve(brake) * dt
            else:
                self.current_speed -= self.sign(self.current_speed) * self.coast_decel * dt
                if abs(self.current_speed) < self.coast_decel * dt:
                    self.current_speed = 0.0

        # ── 4. Boost Nitro Integration ───────────────────────────────────────
        boosting = False
        if boost_requested and self.boost_meter > 0.0 and self.current_speed > 0.0:
            boosting = True
            self.boost_meter -= dt / self.boost_duration
            if self.boost_meter < 0.0:
                self.boost_meter = 0.0
        else:
            self.boost_meter = min(1.0, self.boost_meter + dt / self.boost_regen_time)

        top_speed = self.forward_max * (self.boost_multiplier if boosting else 1.0)

        # Clamp speed limits
        if self.current_speed > top_speed:
            # Gentle drag back to non-boost top speed if boost ended
            self.current_speed = max(top_speed, self.current_speed - self.decel_rate * dt)
        elif self.current_speed < -self.reverse_max:
            self.current_speed = -self.reverse_max

        # ── 5. Lateral Dynamics (Speed-sensitive Steering & Drift) ───────────
        speed_ratio = abs(self.current_speed) / self.forward_max
        steer_gain = self.lerp(self.steer_gain_low_speed, self.steer_gain_high_speed, speed_ratio)

        drift_active = False
        if drift_requested and (abs(self.current_speed) > 0.1):
            drift_active = True
            steer_gain *= self.drift_steer_multiplier
            self.current_speed *= self.drift_traction_loss

        angular_z = steer * steer_gain

        # ── 6. Publish Twist & HUD ───────────────────────────────────────────
        self.publish_twist(self.current_speed, angular_z)

        # Gear calculation
        gear = "D"
        if self.current_speed < -0.05:
            gear = "R"
        elif abs(self.current_speed) < 0.05 and not throttle and not brake:
            gear = "N"

        self.publish_hud(self.current_speed, self.boost_meter, drift_active, gear)

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        """Construct and publish the Twist command."""
        if Twist is None:
            return
        msg = Twist()
        msg.linear.x = round(float(linear_x), 4)
        msg.angular.z = round(float(angular_z), 4)
        self.cmd_vel_pub.publish(msg)

    def publish_hud(self, speed_mps: float, boost_meter: float, drift_active: bool, gear: str) -> None:
        """Construct and publish the HUD JSON telemetry String."""
        payload = {
            "speed_mps": round(float(speed_mps), 4),
            "boost_meter": round(float(boost_meter), 4),
            "boost_percent": round(float(boost_meter) * 100.0, 1),
            "drift_active": bool(drift_active),
            "gear": str(gear),
            "armed": bool(self.armed),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.hud_pub.publish(msg)


def main(args=None):
    node = Ps5NfsTeleop()
    run_node(node, fallback_period=0.02)


if __name__ == "__main__":
    main()
