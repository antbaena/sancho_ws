import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_human_face_recognition',
            executable='camera',
            name='camera',
            output='screen',
            emulate_tty=True,     
        ),
        Node(
            package='py_human_face_recognition',
            executable='detector',
            name='detector',
            output='screen',
            prefix="xterm -hold -e",
            emulate_tty=True,
        ),
        Node(
            package='py_human_face_recognition',
            executable='recognizer',
            name='recognizer',
            output='screen',
            prefix="xterm -hold -e",
            emulate_tty=True,
        ),
        Node(
            package='py_human_face_recognition',
            executable='logic',
            name='logic',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='py_human_face_recognition',
            executable='gui',
            name='gui',
            output='screen',
            prefix="xterm -hold -e",
            emulate_tty=True,
        ),
        Node(
            package='py_hri_voice_recognition',
            executable='audio_direction',
            name='audio_direction',
            output='screen',
            prefix="xterm -hold -e",
            emulate_tty=True,
        )
    ])