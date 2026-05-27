"""
real.launch.py
──────────────
Launch file for Loki AUV real hardware (pool test).

Differences from sim.launch.py:
  - No ros_tcp_endpoint (no Unity)
  - Adds robot_localization EKF for sensor fusion
  - Adds hw_bridge for ESP32 communication (when ready)

Usage:
  ros2 launch loki_bringup real.launch.py
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:

    pid_params = PathJoinSubstitution([
        FindPackageShare("loki_control"),
        "config",
        "pid_params.yaml",
    ])

    ekf_params = PathJoinSubstitution([
        FindPackageShare("loki_bringup"),
        "config",
        "ekf.yaml",
    ])

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[ekf_params],
        remappings=[("odometry/filtered", "/odometry/filtered")],
    )

    controller = Node(
        package="loki_control",
        executable="loki_controller",
        name="loki_controller",
        output="screen",
        parameters=[pid_params],
    )

    monitor = Node(
        package="loki_monitor",
        executable="monitor",
        name="loki_monitor",
        output="screen",
    )

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
    )

    return LaunchDescription([
        ekf_node,
        TimerAction(period=3.0, actions=[controller]),
        TimerAction(period=3.0, actions=[monitor]),
        TimerAction(period=3.0, actions=[foxglove]),
        # hw_bridge added here when ESP32 ready
        # TimerAction(period=3.0, actions=[hw_bridge]),
    ])
