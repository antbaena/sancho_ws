import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_human_face_recognition',
            executable='head',
            name='head',
            output='screen',
            prefix="xterm -hold -e",
            emulate_tty=True,     
        ),
        Node(
            package='py_human_face_recognition',
            executable='logicmix',
            name='logic',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='py_human_face_recognition',
            executable='detectorNoCNN',
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
            executable='body',
            name='body',
            output='screen',
            prefix="xterm -hold -e",
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
            package='interbotix_xs_sdk',
            executable='xs_sdk',
            name='interbotix_xs_sdk',
            output='screen',
            namespace='wxxms',
            prefix="xterm -hold -e",
            parameters=[{
                'motor_configs': os.path.join(get_package_share_directory("interbotix_xsturret_control"), 'config', 'wxxms.yaml'),
                'mode_configs': os.path.join(get_package_share_directory("interbotix_xsturret_control"), 'config', 'modes.yaml'),
                'load_configs': False
            }]
        ),
        Node(
            package = "image_proc",
            executable = "rectify_node",
            name = "rectify_node",
            namespace = "camera",
            output = "screen",
            remappings = [
              ("image", "/image_raw"),
              ("camera_info", "/camera_info"),
              ("/camera/image_rect", "/camera/color/image_raw")
            ],
        )
    ])