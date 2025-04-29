from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur3e_rg2").to_moveit_configs()

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
