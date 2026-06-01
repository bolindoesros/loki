#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch file for Cerulean Sonar Tracker650 ROS2 driver.
    
    Launches two nodes:
    1. receiver: Communicates with DVL hardware via Ethernet UDP
    2. dvl_republisher: Converts raw DVL data to ROS2 standard messages
    
    Tracker 650
    ↓ UDP
    tracker650_receiver
    ↓ /dvl/raw_sentence
    tracker650_republisher
    ↓ /dvl/twist
    robot_localization EKF

    This makes debugging easier. Separating errors caused by ethernet port and parser. 

    Parameters are loaded from tracker650_params.yaml.
    """
    ld = LaunchDescription()

    # Get the path to the parameter file
    pkg_share = get_package_share_directory('loki_hardware_dvl')
    params_file = os.path.join(pkg_share, 'params', 'tracker650_params.yaml')

    # DVL Publisher Node - Hardware interface
    dvl_node = Node(
        package='loki_hardware_dvl',
        executable='tracker650_receiver',
        name='receiver',
        namespace='dvl',
        output='screen',
        parameters=[params_file],
        emulate_tty=True,
    )

    # DVL Republisher Node - Data conversion
    dvl_repub_node = Node(
        package='loki_hardware_dvl',
        executable='tracker650_republisher',
        name='republisher',
        namespace='dvl',
        output='screen',
        parameters=[params_file],
        emulate_tty=True,
    )

    ld.add_action(dvl_node)
    ld.add_action(dvl_repub_node)

    return ld