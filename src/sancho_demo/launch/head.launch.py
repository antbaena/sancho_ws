import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    head_node = Node(
        package='sancho_demo',
        executable='head_demo',
        name='head_demo',
        output='screen',
        prefix="xterm -hold -e",
        emulate_tty=True,
    )
    return LaunchDescription([head_node])

