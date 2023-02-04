import os
from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    is_sim = DeclareLaunchArgument("is_sim", default_value="false")
    prefix = DeclareLaunchArgument("prefix", default_value="")

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'ignore_timestamp': False,
            'frame_prefix': LaunchConfiguration('prefix'),
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('minibot_description'),
                        'urdf',
                        'robot.urdf.xacro',
                    ]),
                    ' is_sim:=', LaunchConfiguration('is_sim'),
                    ' prefix:=', LaunchConfiguration('prefix'),
                ]),
        }]
    )

    ld.add_action(is_sim)
    ld.add_action(prefix)
    ld.add_action(rsp_node)

    return ld