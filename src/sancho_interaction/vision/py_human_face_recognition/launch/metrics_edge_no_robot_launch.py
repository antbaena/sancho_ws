from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="py_human_face_recognition",
                executable="metrics",
                name="metrics",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
            ),
            Node(
                package='py_human_face_recognition',
                executable='camera',
                name='camera',
                output='screen',
                prefix="xterm -hold -e",
                emulate_tty=True,     
            ),
        ]
    )
