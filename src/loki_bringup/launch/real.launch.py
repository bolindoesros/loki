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
        FindPackageShare("auv_localization"),
        "config",
        "ekf.yaml",
    ])

    # hardware drivers
    imu_node = Node(
        package="ahrs_orientation",
        executable="ahrs_node",
        name="ahrs_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    dvl_node = Node(
        package="tracker650_ros_driver",
        executable="tracker650_node",
        name="tracker650_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    esp32_node = Node(
        package="esp32_bench",
        executable="esp32_node",
        name="esp32_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    # localization
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[ekf_params],
        respawn=True,
        respawn_delay=2.0,
    )

    # control
    controller = Node(
        package="loki_control",
        executable="loki_controller",
        name="loki_controller",
        output="screen",
        parameters=[pid_params],
        respawn=True,
        respawn_delay=2.0,
    )

    # monitoring
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
        # hardware drivers
        imu_node,
        dvl_node,
        esp32_node,
        # EKF
        TimerAction(period=2.0, actions=[ekf_node]),
        # control
        TimerAction(period=4.0, actions=[controller]),
        # monitoring and foxglove bridge
        TimerAction(period=6.0, actions=[monitor]),
        TimerAction(period=6.0, actions=[foxglove]),
    ])