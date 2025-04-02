#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ajusta este nombre de paquete y ruta al archivo de parámetros si tienes uno.
    slam_params_file = os.path.join(
        get_package_share_directory('my_slam_config'),
        'config',
        'slam_toolbox.yaml'
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',  # Usa 'async_slam_toolbox_node' si prefieres el modo asíncrono
            name='slam_toolbox',
            output='screen',
            parameters=[
                # Archivo de parámetros (opcional)
                slam_params_file,
                # Parámetros adicionales/override
                {
                    'use_sim_time': False,
                    'slam_toolbox.scan_topic': '/scan_1st'
                }
            ],
        )
    ])
