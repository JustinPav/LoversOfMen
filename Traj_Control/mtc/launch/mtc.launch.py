import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Get absolute path to kinematics.yaml in mtc/config/
    kinematics_yaml_path = os.path.join(
        get_package_share_directory("mtc"),
        "config",
        "kinematics.yaml"
    )

    # Inject it via MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("ur3e_rg2")
        .robot_description_kinematics(file_path=kinematics_yaml_path)
        .to_moveit_configs()
    )

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability",
        "allow_trajectory_execution": True,
    }

    mtc_task = Node(
        package="mtc",
        executable="main",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities
        ]
    )

    return LaunchDescription([
        mtc_task,
    ])
