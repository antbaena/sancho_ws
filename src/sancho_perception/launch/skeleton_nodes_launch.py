
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    movenet_inference_node = Node(
        package='sancho_perception',
        executable='movenet_inference_node',
        name='movenet_inference_node',
        output='screen',
        prefix="xterm -hold -e",
        emulate_tty=True,
    )
    
    movenet_postprocessing_node = Node(
        package='sancho_perception',
        executable='movenet_postprocessing_node',
        name='movenet_postprocessing_node',
        output='screen',
        prefix="xterm -hold -e",
        emulate_tty=True,
    )
    
    group_detection_node = Node(
        package='sancho_perception',
        executable='group_detection_node',
        name='group_detection_node',
        output='screen',
        prefix="xterm -hold -e",
        emulate_tty=True,
    ) 
    depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',  # Verifica el nombre del ejecutable según la documentación
        name='depthimage_to_laserscan',
        parameters=[{
            'output_frame': 'base_link',   # Ajusta según tu configuración de TF
            'range_min': 0.1,              # Rango mínimo (ajusta según tu sensor)
            'range_max': 10.0,             # Rango máximo (ajusta según tu sensor)
            'scan_height': 5,              # Número de filas de la imagen a usar para la conversión
            # Otros parámetros opcionales:
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        remappings=[
            # Se remapean los nombres de los topics internos a los de tu sensor:
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('depth', '/camera/depth/image_raw'),
            ('scan', '/scan_camera')
        ],
        output='screen',
        prefix="xterm -hold -e",
        emulate_tty=True,
    )   

    return LaunchDescription([
        movenet_inference_node,
        movenet_postprocessing_node,
        group_detection_node,
        depthimage_to_laserscan
    ])
