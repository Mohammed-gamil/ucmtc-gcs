"""Compatibility helpers for rover ROS2 nodes.

The workspace environment does not include rclpy, so this module provides a
small fallback implementation that keeps the rover nodes importable and lets the
publish/timer logic run in a local simulation mode.
"""

from __future__ import annotations

import time
from typing import Any

ROS_AVAILABLE: bool
Node: Any
String: Any
rclpy: Any

try:  # pragma: no cover - real rover runtime only.
    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from std_msgs.msg import String as _String

    ROS_AVAILABLE = True
    Node = _Node
    String = _String
    rclpy = _rclpy
except ImportError:  # pragma: no cover - used in this workspace.
    ROS_AVAILABLE = False

    class String:
        """Fallback std_msgs/String that mirrors the real message interface."""

        def __init__(self, data: str = ""):
            self.data = data

        def __repr__(self) -> str:
            return f"String(data={self.data!r})"

        def __eq__(self, other: object) -> bool:
            if isinstance(other, String):
                return self.data == other.data
            return NotImplemented

        def __hash__(self) -> int:
            return hash(self.data)

    class _Logger:
        """Fallback logger matching the rclpy.impl.rcutils_logger interface."""

        def __init__(self, name: str):
            self.name = name

        def info(self, message: str) -> None:
            print(f"[INFO] [{self.name}] {message}")

        def warning(self, message: str) -> None:
            print(f"[WARN] [{self.name}] {message}")

        warn = warning  # rclpy Logger exposes both .warn() and .warning()

        def error(self, message: str) -> None:
            print(f"[ERROR] [{self.name}] {message}")

        def debug(self, message: str) -> None:
            print(f"[DEBUG] [{self.name}] {message}")

        def fatal(self, message: str) -> None:
            print(f"[FATAL] [{self.name}] {message}")

        def get_child(self, name: str) -> "_Logger":
            """Return a child logger (mirrors rclpy)."""
            return _Logger(f"{self.name}.{name}")

    class _FallbackPublisher:
        def __init__(self, topic_name: str):
            self.topic_name = topic_name
            self.last_message: Any = None

        def publish(self, message: Any) -> None:
            self.last_message = message

    class _FallbackClock:
        """Minimal clock stub matching rclpy.clock.Clock used by some nodes."""

        def now(self):
            """Return a time-like object with nanoseconds."""
            return _FallbackTime(int(time.time() * 1e9))

    class _FallbackTime:
        def __init__(self, nanoseconds: int = 0):
            self.nanoseconds = nanoseconds

        def seconds_nanoseconds(self) -> tuple[int, int]:
            sec = self.nanoseconds // 1_000_000_000
            nsec = self.nanoseconds % 1_000_000_000
            return (sec, nsec)

        def __sub__(self, other: "_FallbackTime") -> "_FallbackTime":
            return _FallbackTime(self.nanoseconds - other.nanoseconds)

    class Node:
        """Fallback Node that mirrors the rclpy.node.Node API surface
        used by the rover stack, enabling import and local simulation
        without a live ROS2 installation.
        """

        def __init__(self, name: str):
            self._name = name
            self._logger = _Logger(name)
            self._clock = _FallbackClock()
            self._timers: list[dict[str, Any]] = []
            self._subscriptions: list[dict[str, Any]] = []
            self._publishers: list[_FallbackPublisher] = []

        # ── Parameters ──

        def declare_parameter(self, name: str, default_value=None, descriptor=None):
            """Declare a parameter with optional default value (mirrors rclpy)."""
            if not hasattr(self, "_parameters"):
                self._parameters: dict = {}
            if name not in self._parameters:
                self._parameters[name] = default_value
            return self._parameters[name]

        def get_parameter(self, name: str):
            """Return a parameter value wrapper (mirrors rclpy.node.Node.get_parameter)."""
            if not hasattr(self, "_parameters"):
                self._parameters = {}
            value = self._parameters.get(name)
            return _FallbackParameterValue(value)

        def set_parameters(self, params):
            """Accept parameter updates (no-op in fallback)."""

        # ── Name / Identity ──
        def get_name(self) -> str:
            """Return the node name (mirrors rclpy.node.Node.get_name)."""
            return self._name

        def get_namespace(self) -> str:
            """Return the node namespace (always '/' in fallback)."""
            return "/"

        def get_fully_qualified_name(self) -> str:
            """Return namespace + name."""
            return f"/{self._name}"

        # ── Logger / Clock ──

        def get_logger(self):
            return self._logger

        def get_clock(self):
            return self._clock

        # ── Pub / Sub / Timer ──

        def create_publisher(self, message_type, topic_name: str, qos_profile):
            publisher = _FallbackPublisher(topic_name)
            self._publishers.append(publisher)
            return publisher

        def create_subscription(self, message_type, topic_name: str, callback, qos_profile):
            subscription = {"topic_name": topic_name, "callback": callback}
            self._subscriptions.append(subscription)
            return subscription

        def create_timer(self, period_sec: float, callback):
            timer = {
                "period_sec": period_sec,
                "callback": callback,
                "next_fire": time.monotonic() + period_sec,
            }
            self._timers.append(timer)
            return timer

        # ── Lifecycle ──

        def destroy_node(self) -> None:
            self._timers.clear()
            self._subscriptions.clear()
            self._publishers.clear()

        def destroy_publisher(self, publisher) -> None:
            """Destroy a single publisher (mirrors rclpy)."""
            try:
                self._publishers.remove(publisher)
            except ValueError:
                pass

        def destroy_subscription(self, subscription) -> None:
            """Destroy a single subscription (mirrors rclpy)."""
            try:
                self._subscriptions.remove(subscription)
            except ValueError:
                pass

        def destroy_timer(self, timer) -> None:
            """Destroy a single timer (mirrors rclpy)."""
            try:
                self._timers.remove(timer)
            except ValueError:
                pass

        # ── Simulation spin ──

        def _spin_once(self) -> None:
            now = time.monotonic()
            for timer in self._timers:
                if now >= timer["next_fire"]:
                    timer["next_fire"] = now + timer["period_sec"]
                    timer["callback"]()

    class _FallbackParameterValue:
        """Minimal parameter value wrapper matching rclpy.parameter.Parameter."""

        def __init__(self, value=None):
            self._value = value

        def get_parameter_value(self) -> "_FallbackParameterValue":
            """Return self so .get_parameter_value().double_value etc. work."""
            return self

        @property
        def double_value(self) -> float:
            try:
                return float(self._value) if self._value is not None else 0.0
            except (TypeError, ValueError):
                return 0.0

        @property
        def integer_value(self) -> int:
            try:
                return int(self._value) if self._value is not None else 0
            except (TypeError, ValueError):
                return 0

        @property
        def bool_value(self) -> bool:
            return bool(self._value)

        @property
        def string_value(self) -> str:
            return str(self._value) if self._value is not None else ""

    class _RclpyStub:
        """Fallback rclpy module stub with .ok() state tracking to prevent
        double-init and double-shutdown crashes that mirror real rclpy.
        """

        def __init__(self):
            self._initialized = False

        def ok(self) -> bool:
            """Return True when the ROS context is active (mirrors rclpy.ok)."""
            return self._initialized

        def init(self, *, args=None, context=None) -> None:
            self._initialized = True

        def shutdown(self, *, context=None) -> None:
            self._initialized = False

        def spin(self, node) -> None:
            try:
                while True:
                    node._spin_once()
                    time.sleep(0.05)
            except KeyboardInterrupt:
                return None

        def spin_once(self, node, *, timeout_sec: float | None = None) -> None:
            node._spin_once()
            if timeout_sec is not None and timeout_sec > 0:
                time.sleep(min(timeout_sec, 0.05))

    rclpy = _RclpyStub()


def run_node(node: Node, fallback_period: float = 0.1):
    """Run a node in either a real ROS runtime or the local fallback."""

    if ROS_AVAILABLE:
        initialized_here = ensure_ros_initialized(args=None)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if initialized_here or getattr(node, "_owns_ros_context", False):
                try:
                    if hasattr(rclpy, "ok") and rclpy.ok():
                        rclpy.shutdown()
                except Exception:
                    pass
        return

    try:
        while True:
            node._spin_once()
            time.sleep(fallback_period)
    except KeyboardInterrupt:
        return None
    finally:
        node.destroy_node()


def ensure_ros_initialized(args=None) -> bool:
    """Initialize ROS only when it is not already active.

    Returns True if this call performed the initialization.
    """
    if not ROS_AVAILABLE:
        return False
    try:
        if hasattr(rclpy, "ok") and rclpy.ok():
            return False
        rclpy.init(args=args)
        return True
    except RuntimeError:
        # Already initialized — swallow "Context.init() must only be called once"
        return False


__all__ = ["Node", "ROS_AVAILABLE", "String", "ensure_ros_initialized", "rclpy", "run_node"]
