#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Directories for launch files
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # World file
    world = os.path.join(
        get_package_share_directory('nikiro_gazebo'),
        'worlds',
        'nikiro.world'
    )

    # Gazebo server command
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Gazebo client command
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add actions to launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    return ld
