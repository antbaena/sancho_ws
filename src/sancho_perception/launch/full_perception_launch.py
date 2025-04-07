
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
    
    group_navigation_node = Node(
        package='sancho_perception',
        executable='group_navigation_node',
        name='group_navigation_node',
        output='screen',
        prefix="xterm -hold -e",
        emulate_tty=True,
    )
    return LaunchDescription([
        movenet_inference_node,
        movenet_postprocessing_node,
        group_detection_node,
        group_navigation_node
    ])
