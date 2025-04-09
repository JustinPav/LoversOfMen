from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share_dir = get_package_share_directory('mtc')
    config_dir = os.path.join(package_share_dir, 'config')

    moveit_config = MoveItConfigsBuilder("ur3e") \
        .robot_description_kinematics(os.path.join(config_dir, "kinematics.yaml")) \
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"], default_planning_pipeline="chomp") \
        .to_moveit_configs()

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
 
    return LaunchDescription(\
        [
        mtc_task,
        ]
    )
