"""
  OdometryPublisher.cs publishes to /odometry/filtered
  ActuatorBridge.cs subscribes to /cmd/thruster, /cmd/elevator, /cmd/rudder
  ros2 launch loki_bringup sim.launch.py
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description() -> LaunchDescription:

    pid_params = os.path.join(
        get_package_share_directory("loki_control"),
        "config",
        "pid_params.yaml",
    )

    ros_tcp_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        name="ros_tcp_endpoint",
        output="screen",
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
        ros_tcp_endpoint,
        TimerAction(period=2.0, actions=[controller]),
        TimerAction(period=3.0, actions=[monitor]),
        TimerAction(period=3.0, actions=[foxglove]),
    ])
