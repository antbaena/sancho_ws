from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='interbotix_xs_sdk',
            executable='xs_sdk',
            name='interbotix_xs_sdk',
            #output='screen',
            namespace='wxxms',
            #prefix="xterm -hold -e",
            parameters=[{
                'motor_configs': os.path.join(get_package_share_directory("interbotix_xsturret_control"), 'config', 'wxxms.yaml'),
                'mode_configs': os.path.join(get_package_share_directory("interbotix_xsturret_control"), 'config', 'modes.yaml'),
                'load_configs': False
            }]
        )
    ])
