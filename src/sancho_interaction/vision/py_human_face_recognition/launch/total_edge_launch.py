from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # HOKUYO
    urg_node2_launch = Node(
        package="urg_node2",
        executable="urg_node2_node",
        name="urg_node2",
        output="screen",
        prefix="xterm -hold -e",
    )

    # URDF
    simbot_urdf_launch_file = os.path.join(
        get_package_share_directory("robot_description"),
        "launch",
        "simbot_urdf.launch.py",
    )

    simbot_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simbot_urdf_launch_file)
    )

    # ROS2ARIA
    ros2aria_node = Node(
        package="ros2aria",
        executable="ros2aria",
        name="ros2aria",
        output="screen",
        prefix="xterm -hold -e",
    )
    return LaunchDescription(
        [
            urg_node2_launch,
            simbot_urdf_launch,
            ros2aria_node,
            Node(  # MOVEMENT
                package="py_human_face_recognition",
                executable="head",
                name="head",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
            ),
            Node(
                package="py_human_face_recognition",
                executable="body",
                name="body",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
            ),
            Node(
                package="interbotix_xs_sdk",
                executable="xs_sdk",
                name="interbotix_xs_sdk",
                output="screen",
                namespace="wxxms",
                prefix="xterm -hold -e",
                parameters=[
                    {
                        "motor_configs": os.path.join(
                            get_package_share_directory("interbotix_xsturret_control"),
                            "config",
                            "wxxms.yaml",
                        ),
                        "mode_configs": os.path.join(
                            get_package_share_directory("interbotix_xsturret_control"),
                            "config",
                            "modes.yaml",
                        ),
                        "load_configs": False,
                    }
                ],
            ),
            Node(  # AUDIO DIRECTION
                package="py_hri_voice_recognition",
                executable="microphone_capturer",
                name="microphone",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
            ),
            Node(  # AUDIO INTERACTION
                package="py_hri_voice_recognition",
                executable="assistant",
                name="assistant",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
            ),
            Node(
                package="py_hri_voice_recognition",
                executable="helper",
                name="assistant_helper",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
            ),
            
            Node(
                package="py_human_face_recognition",
                executable="logic",
                name="logic",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="py_human_face_recognition",
                executable="gui",
                name="gui",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
            ),  # CAMERA Se podria meter el rectificar en edge
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
            Node(
                package="sancho_web",
                executable="web",
                name="web",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
            ),
            
            Node(
                package="py_human_face_recognition",
                executable="camera_parameters",
                name="camera_parameters",
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
            ),  # Hay que meter usb_cam con sus parameters
            Node(
                package="py_hri_voice_recognition",
                executable="speaker_controller",
                name="speaker_controller",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
            ),
        ]
    )
