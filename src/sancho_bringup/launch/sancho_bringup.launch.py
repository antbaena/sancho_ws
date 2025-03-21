import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ranger_bringup_path = get_package_share_directory('ranger_bringup')

    # Cargar URDF desde sancho_description
    urdf_file_path = os.path.join(
        get_package_share_directory('sancho_description'),
        'urdf',
        'sancho_ranger.urdf'
    )

    # Lanzar LIDAR (ejemplo con Hokuyo)
    hokuyo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('urg_node2'), 'launch'),
            '/urg_node2_2lidar.launch.py'])
    )

    # Lanzar cámara Orbbec Gemini 330
    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('orbbec_camera'), 'launch'),
            '/gemini_330_series.launch.py'])
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

        # Cámara Astra
        # Node(
        #      package='astra_camera',
        #      executable='astra_camera_node',
        #      name='astra_camera_node',
        #      output='screen'
        #  ),

        # Cámara USB
        # Node(
        #     package="usb_cam",
        #     executable="usb_cam_node_exe",
        #     name="usb_cam_node",
        #     output="screen",
        #     prefix="xterm -hold -e",
        #     emulate_tty=True,
        #     parameters=[
        #         os.path.join(
        #             get_package_share_directory("sancho_bringup"),
        #             "config",
        #             "params_low.yaml",
        #         )
        #     ],
        # ),

        # Incluir el launch de la base móvil
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(ranger_bringup_path, 'launch', 'ranger_mini_v2.launch.xml'))
        ),

        # Iniciar sensores (LiDAR, cámaras, etc.)
        hokuyo_launch,
        # orbbec_camera_launch,
    ])
