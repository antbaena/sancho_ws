from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_hri_voice_recognition',
            executable='assistant',
            name='assistant',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='py_hri_voice_recognition',
            executable='helper',
            name='assistant_helper',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='py_hri_voice_recognition',
            executable='google_stt',
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