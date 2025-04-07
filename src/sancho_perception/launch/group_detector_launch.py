
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
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

        group_detection_node,
        group_navigation_node
    ])
