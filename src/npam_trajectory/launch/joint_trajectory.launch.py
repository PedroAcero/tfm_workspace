from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Cargar kinematics.yaml desde ur_moveit_config
    kinematics_yaml = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'config',
        'kinematics.yaml'
    )

    return LaunchDescription([
        Node(
            package='npam_trajectory',
            executable='joint_planner',
            # name='joint_planner_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                kinematics_yaml,
            ]
        )
    ])
