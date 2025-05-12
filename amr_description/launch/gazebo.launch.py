#!/usr/bin/env python3

from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable, AppendEnvironmentVariable

def generate_launch_description():
    amr_bot_path = get_package_share_directory('amr_description')
    nikiro_gazebo_path = get_package_share_directory('nikiro_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    world_file = LaunchConfiguration("world_file", default=join(nikiro_gazebo_path, 'worlds', 'nikiro.world'))
    
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py"))
    )

    spawn_amr_bot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(amr_bot_path, "launch", "amr_bot_gazebo_spawn.launch.py")),
    )

    return LaunchDescription([
        AppendEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=join(amr_bot_path, "models")
        ),
        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH',
            value="/usr/share/gazebo-11:" + join(amr_bot_path, "worlds")
        ),
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        gazebo,
        spawn_amr_bot_node
    ])

