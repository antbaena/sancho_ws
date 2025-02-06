import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ranger_bringup_path = get_package_share_directory('sancho_bringup')

    # Cargar URDF desde sancho_description
    urdf_file_path = os.path.join(
        get_package_share_directory('sancho_description'),
        'urdf',
        'sancho.urdf'
    )

    # Lanzar LIDAR (ejemplo con Hokuyo)
    hokuyo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('urg_node2'), 'launch'),
            '/urg_node2_2lidar.launch.py'])
    )

    return LaunchDescription([
        # Publicar estado del robot desde URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file_path).read()}]
        ),

        # Publicar estado de articulaciones si es necesario
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Incluir el launch de la base móvil
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ranger_bringup_path, 'launch', 'ranger_mini_v2.launch.xml')
            )
        ),

        # Iniciar sensores (LiDAR, cámaras, etc.)
        hokuyo_launch,
    ])
