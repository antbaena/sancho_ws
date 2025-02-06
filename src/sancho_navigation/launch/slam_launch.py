import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ranger_bringup_path = get_package_share_directory('ranger_bringup')

    return LaunchDescription([
        # Incluir el launch de la base usando AnyLaunchDescriptionSource
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(ranger_bringup_path, 'launch', 'ranger_mini_v2.launch.xml'))
        ),

        # Nodo de SLAM Toolbox (para generar el mapa)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'map_update_interval': 0.5
            }]
        ),



        # Nodo del LiDAR (para recibir los datos del sensor)
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='hokuyo_driver',
            parameters=[{'serial_port': '/dev/ttyACM0',
                         'frame_id': 'laser',
                          'angle_min': -1.57,  # Limita el ángulo mínimo del escáner
                          'angle_max': 1.57}],
            output='screen'
        ),

        # Nodo de TF para la transformación entre la base y el LiDAR
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.2', '0.0', '0.1', '0', '0', '0', '1', 'base_link', 'laser']
        )
    ])
