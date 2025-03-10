from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

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
                package="py_human_face_recognition",
                executable="converter",
                name="converter",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
            ),
            Node(  # CAMERA
                package="image_proc",
                executable="rectify_node",
                name="rectify_node",
                namespace="camera",
                output="screen",
                remappings=[
                    ("image", "/camera/color/image_rgb"),
                    ("camera_info", "/camera_info"),
                    ("/camera/image_rect", "/camera/color/image_raw"),
                ],
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="usb_cam_node",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[
                    os.path.join(
                        get_package_share_directory("usb_cam"),
                        "config",
                        "params_low.yaml",
                    )
                ],
            ),
        ]
    )
