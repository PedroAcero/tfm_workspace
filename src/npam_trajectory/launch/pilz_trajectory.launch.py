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
    # En este caso, la velocidad se va a definir como:
    # velocity_scaling = F del g-code / max_cartesian_speed
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

    # Argumento: max_cartesian_speed (mm/s). Valor sin calibrar: 1000 mm/s
    max_cartesian_speed_arg = DeclareLaunchArgument(
        'max_cartesian_speed',
        default_value='1000.0',
        description='Velocidad cartesiana media medida empíricamente (mm/s). '
                    'Ver flujo de calibración en pilz_planner.cpp.'
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
                'max_cartesian_speed': LaunchConfiguration('max_cartesian_speed'),
            }
        ]
    )

    return LaunchDescription([
        gcode_directory_arg,
        gcode_filename_arg,
        max_cartesian_speed_arg,
        pilz_planner_node
    ])
