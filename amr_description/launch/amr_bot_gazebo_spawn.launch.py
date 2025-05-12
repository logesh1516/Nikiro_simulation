#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PythonExpression

from launch_ros.actions import Node

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    amr_bot_path = get_package_share_directory('amr_description')
    
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    odometry_source = LaunchConfiguration("odometry_source", default="world")
    robot_namespace = LaunchConfiguration("robot_namespace", default='amr')
    
    xacro_path = join(amr_bot_path, 'urdf', 'amr.xacro')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', xacro_path,
                    ' sim_gazebo:=', "true",
                    ' odometry_source:=', odometry_source,
                    ' robot_namespace:=', robot_namespace,
                    ])}],
        remappings=[
            ('/joint_states', PythonExpression(['"', robot_namespace, '/joint_states"'])),
        ]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', "/robot_description",
            '-entity', PythonExpression(['"', robot_namespace, '_robot"']),
            '-z', "0.28",
            '-x', position_x,
            '-y', position_y,
            '-R', "0.0",
        ]
    )



    return LaunchDescription([
        DeclareLaunchArgument("position_x", default_value="-0.8"),
        DeclareLaunchArgument("position_y", default_value="-1.4"),
        DeclareLaunchArgument("odometry_source", default_value = odometry_source),
        DeclareLaunchArgument("robot_namespace", default_value = robot_namespace),
        robot_state_publisher,
        spawn_entity
    ])
