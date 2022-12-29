from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    prefix = DeclareLaunchArgument("prefix", default_value="")
    lidar_model = DeclareLaunchArgument("lidar_model", default_value="ydlidar_x2")
    port_name = DeclareLaunchArgument("port_name", default_value="/dev/ttyACM0")
    baudrate = DeclareLaunchArgument("baudrate", default_value="921600")

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('minibot_description'),
            'urdf/robot.urdf.xacro',
        ]),
        ' is_sim:=', 'false',
        ' prefix:=', LaunchConfiguration('prefix'),
        ' lidar_model:=', LaunchConfiguration('lidar_model'),
        ' port_name:=', LaunchConfiguration('port_name'),
        ' baudrate:=', LaunchConfiguration('baudrate'),
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
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
                'joint_state_broadcaster'],
        output='screen'
    )

    load_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
                'base_controller'],
        output='screen'
    )

    ld.add_action(prefix)
    ld.add_action(lidar_model)
    ld.add_action(port_name)
    ld.add_action(baudrate)
    ld.add_action(upload_robot)
    ld.add_action(control_node)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_base_controller)
    return ld