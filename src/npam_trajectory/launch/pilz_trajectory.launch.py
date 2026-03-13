from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Configuración de MoveIt
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

    # =====================================================
    # ARGUMENTOS DEL LAUNCH
    # La velocidad de ejecución se controla mediante los
    # límites de pilz_cartesian_limits.yaml en move_group.
    # velocity_scaling actúa como factor global [0.0-1.0]
    # sobre esos límites. Por defecto 1.0 (100%).
    # =====================================================
    gcode_directory_arg = DeclareLaunchArgument(
        'gcode_directory',
        default_value=os.environ['HOME'] +
        '/workspace/src/npam_trajectory/trayectorias/',
        description='Directorio donde se encuentran los archivos G-code'
    )
    gcode_filename_arg = DeclareLaunchArgument(
        'gcode_filename',
        default_value='short_generated_gcode_medio_estrella_poses.gcode',
        description='Nombre del archivo G-code a ejecutar'
    )
    velocity_scaling_arg = DeclareLaunchArgument(
        'velocity_scaling',
        default_value='1.0',
        description='Factor de escalado de velocidad [0.0-1.0] aplicado '
                    'sobre los límites de pilz_cartesian_limits.yaml'
    )

    # =====================================================
    # NODO: pilz_planner
    # =====================================================

    pilz_planner_node = Node(
        package='npam_trajectory',
        executable='pilz_planner',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {
                'gcode_directory': LaunchConfiguration('gcode_directory'),
                'gcode_filename': LaunchConfiguration('gcode_filename'),
                'velocity_scaling': LaunchConfiguration('velocity_scaling'),
            }
        ]
    )

    return LaunchDescription([
        gcode_directory_arg,
        gcode_filename_arg,
        velocity_scaling_arg,
        pilz_planner_node
    ])
