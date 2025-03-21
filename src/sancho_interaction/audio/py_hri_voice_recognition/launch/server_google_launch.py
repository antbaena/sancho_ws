from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_hri_voice_recognition',
            executable='audio_direction',
            name='direction',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='py_hri_voice_recognition',
            executable='whisper_stt',
            name='stt',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='py_hri_voice_recognition',
            executable='google_tts',
            name='tts',
            output='screen',
            emulate_tty=True,
        )
    ])