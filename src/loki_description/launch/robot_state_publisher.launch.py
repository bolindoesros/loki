from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    urdf_file = os.path.join(
        get_package_share_directory('loki_description'),
        'urdf',
        'Full_Assembly_URDF_Combined.urdf'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]
        )
    ])