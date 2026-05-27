"""
sim.launch.py
─────────────
Launch file for Loki AUV Unity simulation.

Unity side required:
  OdometryPublisher  → publishes to /odometry/filtered
  ActuatorBridge     → subscribes to /cmd/thruster, /cmd/elevator, /cmd/rudder

Usage:
  ros2 launch loki_bringup sim.launch.py

Commands (separate terminal):
  ros2 service call /system/arm std_srvs/srv/SetBool "{data: true}"
  ros2 topic pub --once /target/speed        std_msgs/msg/Float64 "{data: 1.5}"
  ros2 topic pub --once /target/depth        std_msgs/msg/Float64 "{data: 0.8}"
  ros2 topic pub --once /target/heading      std_msgs/msg/Float64 "{data: 0.0}"
  ros2 topic pub --once /target/moving_mass  std_msgs/msg/Float64 "{data: 0.5}"
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
        TimerAction(period=3.0, actions=[controller]),
        TimerAction(period=3.0, actions=[monitor]),
        TimerAction(period=3.0, actions=[foxglove]),
    ])
