#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    # Launch arguments
    use_xterm = LaunchConfiguration('use_xterm')
    prefix_cmd = LaunchConfiguration('prefix')

    # Lifecycle Nodes
    group_detection_node = LifecycleNode(
        package='sancho_perception',
        executable='group_detection_lifecycle_node',
        name='group_detection_lifecycle_node',
        output='screen',
        prefix=prefix_cmd,
        emulate_tty=True,
    )

    group_waypoint_generator_node = LifecycleNode(
        package='sancho_perception',
        executable='group_waypoint_generator_lifecycle_node',
        name='group_waypoint_generator_lifecycle_node',
        output='screen',
        prefix=prefix_cmd,
        emulate_tty=True,
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_sancho',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'group_detection_lifecycle_node',
                'group_waypoint_generator_lifecycle_node'
            ]
        }]
    )

    return LaunchDescription([
        # Declare args
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
            group_detection_node,
            group_waypoint_generator_node,
            lifecycle_manager,
        ]),
    ])
