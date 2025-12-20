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
    # ARGUMENTOS DEL LAUNCH (pueden ser modificados)
    # =====================================================

    # Argumento: velocity_scaling (0.0 - 1.0)
    velocity_scaling_arg = DeclareLaunchArgument(
        'velocity_scaling',
        default_value='0.1',
        description='Factor de escalado de velocidad (0.0-1.0). 0.1 = 10% de velocidad máxima'
    )

    # Argumento: acceleration_scaling (0.0 - 1.0)
    acceleration_scaling_arg = DeclareLaunchArgument(
        'acceleration_scaling',
        default_value='0.1',
        description='Factor de escalado de aceleración (0.0-1.0). 0.1 = 10% de aceleración máxima'
    )

    # Argumento: gcode_directory
    gcode_directory_arg = DeclareLaunchArgument(
        'gcode_directory',
        default_value='/home/pedro/workspace/src/npam_trajectory/trayectorias/',
        description='Directorio donde se encuentran los archivos G-code'
    )

    # Argumento: gcode_filename
    gcode_filename_arg = DeclareLaunchArgument(
        'gcode_filename',
        default_value='short_generated_gcode_medio_estrella_poses.gcode',
        description='Nombre del archivo G-code a ejecutar'
    )

    # =====================================================
    # NODO: cartesian_planner
    # =====================================================

    cartesian_planner_node = Node(
        package='npam_trajectory',
        executable='cartesian_planner',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {
                'velocity_scaling': LaunchConfiguration('velocity_scaling'),
                'acceleration_scaling': LaunchConfiguration('acceleration_scaling'),
                'gcode_directory': LaunchConfiguration('gcode_directory'),
                'gcode_filename': LaunchConfiguration('gcode_filename'),
            }
        ]
    )

    return LaunchDescription([
        # Declarar argumentos
        velocity_scaling_arg,
        acceleration_scaling_arg,
        gcode_directory_arg,
        gcode_filename_arg,
        cartesian_planner_node
    ])
