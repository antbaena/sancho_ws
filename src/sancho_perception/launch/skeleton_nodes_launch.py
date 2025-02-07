
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodo para la detección de pose 2D y publicación de imagen anotada
    skeleton_detection_node = Node(
        package='sancho_perception',
        executable='skeleton_detection_node',
        name='skeleton_detection_node',
        output='screen'
    )

    # Nodo para la transformación de keypoints 2D a 3D y publicación de imagen de profundidad anotada
    skeleton_3d_node = Node(
        package='sancho_perception',
        executable='skeleton_3d_node',
        name='skeleton_3d_node',
        output='screen'
    )
    
    person_pose_publisher_node = Node(
        package='sancho_perception',
        executable='person_pose_publisher_node',
        name='person_pose_publisher_node',
        output='screen'
    )

    return LaunchDescription([
        skeleton_detection_node,
        skeleton_3d_node,
        person_pose_publisher_node
    ])
