#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    # Parámetros de lanzamiento
    prefix_cmd = LaunchConfiguration('prefix')
    audio_node_name = 'audio_player'

    # Nodos lifecycle
    audio_player_node = LifecycleNode(
        namespace='',
        package='sancho_interaction_audio',
        executable='audio_player',
        name=audio_node_name,
        output='screen',
        prefix=prefix_cmd,
        emulate_tty=True,
    )
    # Comandos para configurar los nodos
    configure_audio = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', ["/", audio_node_name], 'configure'],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'prefix',
            default_value='xterm -hold -e' if os.environ.get('DISPLAY') else '',
            description='Prefijo para lanzar nodos en terminal'
        ),

        GroupAction([
            audio_player_node,
            TimerAction(
                period=7.0,
                actions=[
                    configure_audio
                ]
            ),
        ]),
    ])
