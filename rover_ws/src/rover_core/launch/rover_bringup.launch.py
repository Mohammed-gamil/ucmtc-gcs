"""
ROS 2 Python launch file for rover_bringup.

Instantiates and configures the rover stack nodes:
  1. NavigationNode        — GPS/IMU telemetry and positioning
  2. SafetyNode            — Emergency stop and collision detection
  3. VisionNode            — Camera and AI inference for lane/obstacle detection
  4. MotorControlNode      — Drive command heartbeat and command intake
  5. TelemetryAggregatorNode — Canonical payload merger for GCS telemetry

All nodes load parameters from config/params.yaml via FindPackageShare so
the launch file contains no hardcoded absolute paths.

Launch with:
  ros2 launch rover_core rover_bringup.launch.py
Optional overrides:
  ros2 launch rover_core rover_bringup.launch.py use_sim_time:=false log_level:=debug
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Generate the launch description with all rover core nodes."""

    # ── Launch arguments ─────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock (set true when running with Gazebo/Isaac)",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for all rover nodes (debug/info/warn/error/fatal)",
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="Topic name for standard cmd_vel commands",
    )
    use_simulation_arg = DeclareLaunchArgument(
        "use_simulation",
        default_value="true",
        description="Whether to run in simulation mode (true) or talk to physical hardware (false)",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    use_simulation = LaunchConfiguration("use_simulation")

    # ── Shared parameter file ─────────────────────────────────────────────────
    # FindPackageShare resolves to the installed share directory, avoiding
    # hardcoded /home/... paths that break on other machines.
    params_file = PathJoinSubstitution([
        FindPackageShare("rover_core"),
        "config",
        "params.yaml",
    ])

    common_kwargs = {
        "output": "screen",
        "parameters": [
            params_file,
            {"use_sim_time": use_sim_time},
            {"use_simulation": use_simulation},
        ],
        "arguments": ["--ros-args", "--log-level", log_level],
    }

    # ── Nodes ────────────────────────────────────────────────────────────────
    navigation_node = Node(
        package="rover_core",
        executable="navigation_node",
        name="navigation_node",
        **common_kwargs,
    )

    safety_node = Node(
        package="rover_core",
        executable="safety_node",
        name="safety_node",
        respawn=True,
        respawn_delay=2.0,
        **common_kwargs,
    )

    vision_node = Node(
        package="rover_core",
        executable="vision_node",
        name="vision_node",
        **common_kwargs,
    )

    motor_control_node = Node(
        package="rover_core",
        executable="motor_control_node",
        name="motor_control_node",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
            {"use_simulation": use_simulation},
            {"cmd_vel_topic": cmd_vel_topic},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    telemetry_aggregator = Node(
        package="rover_core",
        executable="telemetry_aggregator",
        name="telemetry_aggregator",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
            {"use_simulation": use_simulation},
            {"cmd_vel_topic": cmd_vel_topic},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        cmd_vel_topic_arg,
        use_simulation_arg,
        navigation_node,
        safety_node,
        vision_node,
        motor_control_node,
        telemetry_aggregator,
    ])
