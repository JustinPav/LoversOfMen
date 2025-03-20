from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share_dir = get_package_share_directory('test')
    config_dir = os.path.join(package_share_dir, 'config')

    moveit_config = MoveItConfigsBuilder(robot_name="ur3e", package_name="test")
    moveit_config.robot_description_kinematics(os.path.join(config_dir, "kinematics.yaml"))
    moveit_config.planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
    moveit_config = moveit_config.to_moveit_configs()

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    return LaunchDescription([
        Node(
            package="test",
            executable="mtc_main",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                move_group_capabilities
            ]
        )
    ])
