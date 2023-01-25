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
    lidar_model = DeclareLaunchArgument("lidar_model", default_value="hokuyo")
    lidar_port_name = DeclareLaunchArgument("lidar_port_name", default_value="/dev/ttyHokuyo")
    lidar_baudrate = DeclareLaunchArgument("lidar_baudrate", default_value="57600")
    robot_port_name = DeclareLaunchArgument("robot_port_name", default_value="/dev/ttyArduino")
    robot_baudrate = DeclareLaunchArgument("robot_baudrate", default_value="921600")

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('minibot_description'),
            'urdf/robot.urdf.xacro',
        ]),
        ' is_sim:=', 'false',
        ' lidar_model:=', LaunchConfiguration('lidar_model'),
        ' port_name:=', LaunchConfiguration('robot_port_name'),
        ' baudrate:=', LaunchConfiguration('robot_baudrate'),
        ' prefix:=', LaunchConfiguration('prefix'),

    ])
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([
            FindPackageShare('minibot_bringup'),
            "config",
            "minibot_controllers.yaml"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers
            ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('minibot_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments = {
            'is_sim': 'false',
            'prefix': LaunchConfiguration('prefix'),
            'lidar_model': LaunchConfiguration('lidar_model'),
        }.items()
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    load_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'base_controller'],
        output='screen'
    )

    load_minibot_io_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'minibot_io_controller'],
        output='screen'
    )

    urg_node = Node(
        package="urg_node",
        executable="urg_node_driver",
        parameters=[
            {"serial_port": LaunchConfiguration("lidar_port_name")},
            {"laser_frame_id": "laser_link"},
            {"calibrate_time": False},
            {"publish_intensity": True},
            {"default_user_latency": -0.15},
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        condition=LaunchConfigurationEquals("lidar_model", "hokuyo"),
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_base_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_base_controller,
                on_exit=[load_minibot_io_controller],
            )
        ),
        prefix,
        lidar_model,
        lidar_port_name,
        lidar_baudrate,
        robot_port_name,
        robot_baudrate,
        upload_robot,
        control_node,
        # urg_node,
    ])