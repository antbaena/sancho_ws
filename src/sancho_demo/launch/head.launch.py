import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    head_node = Node(
        package='sancho_demo',
        executable='head_demo',
        name='head_demo',
        output='screen',
        prefix="xterm -hold -e",
        emulate_tty=True,
    )
    interbotix_node =                Node(
                package="interbotix_xs_sdk",
                executable="xs_sdk",
                name="interbotix_xs_sdk",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                namespace="wxxms",
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
            )
        #Test node
    return LaunchDescription([
                              interbotix_node,
                              head_node
                              ])

