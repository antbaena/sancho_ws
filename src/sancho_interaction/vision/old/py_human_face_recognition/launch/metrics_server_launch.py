from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(  # FACE PIPELINE
                package="py_human_face_recognition",
                executable="detector",
                name="detector",
                output="screen",
                parameters=[{"active_cnn": True}, {"show_metrics": True}],
                emulate_tty=True,
            ),
            Node(
                package="py_human_face_recognition",
                executable="recognizer",
                name="recognizer",
                output="screen",
                parameters=[{"show_metrics": True}],
                emulate_tty=True,
            ),
        ]
    )
