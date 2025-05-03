from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_voltage = LaunchConfiguration("tool_voltage")
    onrobot_type = LaunchConfiguration("onrobot_type")
    connection_type = LaunchConfiguration("connection_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # Declare launch arguments
    declare_args = [
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("robot_ip"),
        DeclareLaunchArgument("use_tool_communication", default_value="true"),
        DeclareLaunchArgument("tool_parity", default_value="2"),
        DeclareLaunchArgument("tool_baud_rate", default_value="1000000"),
        DeclareLaunchArgument("tool_voltage", default_value="24"),
        DeclareLaunchArgument("onrobot_type", default_value="rg2"),
        DeclareLaunchArgument("connection_type", default_value="serial"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("prefix", default_value="", description="Prefix for joint names, e.g. 'robot_'"),
    ]

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_robot_driver"),
            "launch",
            "ur_control.launch.py"
        ])),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_tool_communication": use_tool_communication,
            "tool_parity": tool_parity,
            "tool_baud_rate": tool_baud_rate,
            "tool_voltage": tool_voltage,
            "launch_rviz": launch_rviz,
            "use_fake_hardware": use_fake_hardware,
        }.items()
    )

    onrobot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("onrobot_driver"),
            "launch",
            "onrobot_control.launch.py"
        ])),
        launch_arguments={
            "onrobot_type": onrobot_type,
            "connection_type": connection_type,
            "launch_rviz": "false",
            "launch_rsp": "false",
            "use_fake_hardware": use_fake_hardware,
        }.items()
    )

    # Optional: include your rsp.launch.py if it includes any extra robot_description processing
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur3e_rg2_control"),
            "launch",
            "rsp.launch.py"
        ])),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_mock_hardware": use_fake_hardware,
            "mock_sensor_commands": "false",
            "headless_mode": "false",
            "onrobot_type": onrobot_type,
        }.items()
    )

    return LaunchDescription(
        declare_args + [
            rsp,
            ur_control_launch,
            onrobot_control_launch
        ]
    )
