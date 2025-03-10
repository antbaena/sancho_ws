from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
         Node(
            package='py_hri_voice_recognition',
            executable='microphone_capturer',
            name='microphone',
            output='screen',
            emulate_tty=True,
        ),
        
    ])