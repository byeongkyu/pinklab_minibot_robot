import os
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory

from pathlib import Path

def generate_launch_description():
    robot_name = DeclareLaunchArgument("robot_name", default_value="minibot")
    robot_prefix = DeclareLaunchArgument("robot_prefix", default_value="")
    world_name = DeclareLaunchArgument("world_name", default_value="empty.world")

    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                    EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
                                    '/usr/share/gazebo-11/models/:',
                                    str(Path(get_package_share_directory('minibot_description')).parent.resolve()),
                                    ':',
                                    str(Path(get_package_share_directory('minibot_gazebo')).parent.resolve()) + "/minibot_gazebo/models",
                        ])

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'),
            '/launch/gzserver.launch.py'
        ]),
        launch_arguments={
            "verbose": "true",
            "physics": "ode",
            "lockstep": "true",
            "world": PathJoinSubstitution([
                        FindPackageShare('minibot_gazebo'),
                        'worlds',
                        LaunchConfiguration('world_name'),
            ])
        }.items(),
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'),
            '/launch/gzclient.launch.py'
        ]),
    )

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('minibot_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments = {
            'is_sim' : 'true',
            'prefix': LaunchConfiguration('robot_prefix'),
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-timeout', '20.0',
            '-x', '0.0',
            '-y', '0.0',
            '-package_to_model'
        ],
        prefix="bash -c 'sleep 2.0; $0 $@' ",
        parameters=[{
            "use_sim_time": True
        }],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    load_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'base_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_base_controller],
            )
        ),
        gz_resource_path,
        robot_name,
        robot_prefix,
        world_name,
        gz_server,
        gz_client,
        upload_robot,
        spawn_robot,
    ])