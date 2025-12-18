from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    kinematics_yaml = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'config',
        'kinematics.yaml'
    )

    robot_description_semantic = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'srdf',
        'ur.srdf.xacro'
    )

    # Procesar XACRO con los mismos argumentos
    robot_description_content = Command([
        'xacro ',
        os.path.join(
            get_package_share_directory('ur_description'),
            'urdf',
            'ur.urdf.xacro'
        ),
        ' name:=ur',
        ' ur_type:=ur10',
        ' prefix:=""',
    ])

    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        Node(
            package='npam_trajectory',
            executable='cartesian_planner',
            output='screen',
            emulate_tty=True,
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
            ]
        )
    ])
