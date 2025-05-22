#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    # Parámetros de lanzamiento
    prefix_cmd = LaunchConfiguration('prefix')
    manager_node_name = 'interaction_manager'

    # Nodos lifecycle
    interaction_manager_node = LifecycleNode(
        namespace='',
        package='sancho_interaction_manager',
        executable='interaction_manager_node',
        name=manager_node_name,
        output='screen',
        prefix=prefix_cmd,
        emulate_tty=True,
    )
    # Comandos para configurar los nodos
    configure_manager = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', ["/", manager_node_name], 'configure'],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'prefix',
            default_value='xterm -hold -e' if os.environ.get('DISPLAY') else '',
            description='Prefijo para lanzar nodos en terminal'
        ),

        GroupAction([
            interaction_manager_node,
            TimerAction(
                period=5.0,
                actions=[
                    configure_manager
                ]
            ),
        ]),
    ])
