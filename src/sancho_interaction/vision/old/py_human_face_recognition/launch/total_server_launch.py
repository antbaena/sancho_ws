from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            
            
            Node(
                package="py_hri_voice_recognition",
                executable="audio_direction",
                name="direction",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="py_hri_voice_recognition",
                executable="whisper_stt",
                name="stt",
                output="screen",
                emulate_tty=True,
            ),
            Node(  
                package="py_human_face_recognition",
                executable="detector",
                name="detector",
                output="screen",
                parameters=[{"active_cnn": True}],
                emulate_tty=True,
            ),
            Node(
                package="py_human_face_recognition",
                executable="recognizer",
                name="recognizer",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="py_hri_voice_recognition",
                executable="hri_tts",
                name="tts",
                output="screen",
                emulate_tty=True,
            ),  # FACE PIPELINE
        ]
    )
