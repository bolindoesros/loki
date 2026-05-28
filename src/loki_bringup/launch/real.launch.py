"""
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
        respawn=True,
        respawn_delay=2.0,
    )

    controller = Node(
        package="loki_control",
        executable="loki_controller",
        name="loki_controller",
        output="screen",
        parameters=[pid_params],
        respawn=True,
        respawn_delay=2.0,
    )

    monitor = Node(
        package="loki_monitor",
        executable="monitor",
        name="loki_monitor",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        ekf_node,
        #TimerAction(period=1.0, actions=[hw_bridge]),
        TimerAction(period=2.0, actions=[controller]),
        TimerAction(period=4.0, actions=[monitor]),
        TimerAction(period=4.0, actions=[foxglove]),        
    ])
