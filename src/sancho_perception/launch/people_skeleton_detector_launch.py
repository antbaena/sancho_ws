#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnStateTransition

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    sancho_perception_dir = get_package_share_directory('sancho_perception')
    movenet_params_file = os.path.join(sancho_perception_dir, 'config', 'movenet_inference_params.yaml')
    postprocessing_params_file = os.path.join(sancho_perception_dir, 'config', 'movenet_postprocessing_params.yaml')

    # Launch Arguments
    use_xterm = LaunchConfiguration('use_xterm')

    prefix_cmd = LaunchConfiguration('prefix')
    
    movenet_inference_node = LifecycleNode(
        package='sancho_perception',
        executable='movenet_inference_node',
        name='movenet_inference_node',
        output='screen',
        parameters=[movenet_params_file],
        prefix=prefix_cmd,
        emulate_tty=True,
    )
    
    movenet_postprocessing_node = LifecycleNode(
        package='sancho_perception',
        executable='movenet_postprocessing_node',
        name='movenet_postprocessing_node',
        output='screen',
        parameters=[postprocessing_params_file],
        prefix=prefix_cmd,
        emulate_tty=True,
    )

    return LaunchDescription([
        # Opcional: usar xterm o no
        DeclareLaunchArgument(
            'use_xterm',
            default_value='true',
            description='Launch nodes in xterm terminals'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='xterm -hold -e' if os.environ.get('DISPLAY') else '',
            description='Command prefix for launching nodes, e.g., xterm -hold -e'
        ),
        
        GroupAction([
            movenet_inference_node,
            movenet_postprocessing_node,

            # Activar automáticamente el nodo de inferencia
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=movenet_inference_node,
                    start_state='inactive',
                    goal_state='active',
                    entities=[]
                )
            ),
            # Activar automáticamente el nodo de postprocesado
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=movenet_postprocessing_node,
                    start_state='inactive',
                    goal_state='active',
                    entities=[]
                )
            ),
        ]),
    ])
