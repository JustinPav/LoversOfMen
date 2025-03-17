from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('test'), 'config')

    urdf_path = os.path.join(config_dir, 'ur3e_robot.urdf')
    srdf_path = os.path.join(config_dir, 'ur3e_robot.srdf')
    kinematics_path = os.path.join(config_dir, 'kinematics.yaml')
    joint_limits_path = os.path.join(config_dir, 'joint_limits.yaml')
    ompl_path = os.path.join(config_dir, 'ompl_planning.yaml')
    controllers_path = os.path.join(config_dir, 'controllers.yaml')

    return LaunchDescription([
        Node(
            package='test',
            executable='mtc_main',
            output='screen',
            parameters=[
                {'robot_description': open(urdf_path).read()},
                {'robot_description_semantic': open(srdf_path).read()},
                kinematics_path,
                joint_limits_path,
                ompl_path,
                controllers_path,
            ]
        )
    ])
