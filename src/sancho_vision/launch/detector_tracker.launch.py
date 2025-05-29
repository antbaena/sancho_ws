#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    # Parámetros de lanzamiento
    prefix_cmd = LaunchConfiguration('prefix')
    face_node_name = 'face_detector'
    tracker_node_name = 'face_tracker'

    # Nodos lifecycle
    face_detector_node = LifecycleNode(
        namespace='',
        package='sancho_vision',
        executable='face_detector_lifecycle_node',
        name=face_node_name,
        output='screen',
        prefix=prefix_cmd,
        emulate_tty=True,
    )

    face_tracker_node = LifecycleNode(
        namespace='',
        package='sancho_vision',
        executable='face_tracker_lifecycle_node',
        name=tracker_node_name,
        output='screen',
        prefix=prefix_cmd,
        emulate_tty=True,
    )

    # Comandos para configurar los nodos
    configure_face = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', ["/", face_node_name], 'configure'],
        output='screen'
    )

    configure_tracker = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', ["/", tracker_node_name], 'configure'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'prefix',
            default_value='xterm -hold -e' if os.environ.get('DISPLAY') else '',
            description='Prefijo para lanzar nodos en terminal'
        ),
        DeclareLaunchArgument(
            'face_node_name',
            default_value='face_detector_node',
            description='Nombre del nodo de detección'
        ),
        DeclareLaunchArgument(
            'tracker_node_name',
            default_value='face_tracker_node',
            description='Nombre del nodo de seguimiento'
        ),

        GroupAction([
            face_detector_node,
            face_tracker_node,
            TimerAction(
                period=7.0,
                actions=[
                    configure_face,
                    configure_tracker,
                ]
            ),
        ]),
    ])
