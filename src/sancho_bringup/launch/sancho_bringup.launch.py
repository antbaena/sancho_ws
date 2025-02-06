import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Publicador de estado del robot (para TF)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": open(
                os.path.join(
                    os.getenv("COLCON_PREFIX_PATH").split(":")[0],
                    "share",
                    "sancho_description",
                    "urdf",
                    "sancho_ranger.urdf"
                )).read()}]
        ),

        # Nodo de percepción (detección de esqueleto)
        Node(
            package="sancho_perception",
            executable="skeleton_detection_node",
            name="skeleton_detection",
            output="screen",
        ),

        # Nodo de navegación
        Node(
            package="sancho_navigation",
            executable="nav2_bringup",
            name="navigation",
            output="screen",
            parameters=["config/nav2_params_ranger.yaml"]
        )
    ])
