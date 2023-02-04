from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnExecutionComplete, OnProcessStart
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    prefix = DeclareLaunchArgument("prefix", default_value="")
    world_name = DeclareLaunchArgument("world_name", default_value="empty.world")

    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('minibot_gazebo'),
            '/launch/bringup_gazebo.launch.py']
        ),
        launch_arguments = {
            'robot_prefix': LaunchConfiguration('prefix'),
            'world_name': LaunchConfiguration('world_name'),
        }.items()
    )

    return LaunchDescription([
        prefix,
        world_name,
        gazebo_bringup
    ])