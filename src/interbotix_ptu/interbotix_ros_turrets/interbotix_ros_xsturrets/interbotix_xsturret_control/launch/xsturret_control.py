from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value=""),
        DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration("robot_model")),
        DeclareLaunchArgument('base_link_frame', default_value="base_link"),
        DeclareLaunchArgument('use_world_frame', default_value="true"),
        DeclareLaunchArgument('external_urdf_loc', default_value=""),
        DeclareLaunchArgument('use_rviz', default_value="true"),
        DeclareLaunchArgument('motor_configs', default_value=[get_package_share_directory('interbotix_xsturret_control'), '/config/', LaunchConfiguration('robot_model'),'.yaml']),
        DeclareLaunchArgument('mode_configs', default_value=[get_package_share_directory('interbotix_xsturret_control'), '/config/modes.yaml']),
        DeclareLaunchArgument('load_configs', default_value="true"),
        DeclareLaunchArgument('use_sim', default_value="false"),

        DeclareLaunchArgument('xs_sdk_type', default_value="xs_sdk", condition=UnlessCondition(LaunchConfiguration('use_sim'))),
        DeclareLaunchArgument('xs_sdk_type', default_value="xs_sdk_sim", condition=IfCondition(LaunchConfiguration('use_sim'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('interbotix_xsturret_descriptions'), '/launch/xsturret_description.py']),
            launch_arguments={
                'robot_model': LaunchConfiguration("robot_model"),
                'robot_name': LaunchConfiguration("robot_name"),
                'base_link_frame': LaunchConfiguration("base_link_frame"),
                'use_world_frame': LaunchConfiguration("use_world_frame"),
                'external_urdf_loc': LaunchConfiguration("external_urdf_loc"),
                'use_rviz': LaunchConfiguration("use_rviz")
            }.items(),
        ),

        Node(
            package='interbotix_xs_sdk',
            executable='xs_sdk',
            name='xs_sdk',
            output='screen',
            namespace=LaunchConfiguration('robot_name'),
            parameters=[
                {'motor_configs': LaunchConfiguration("motor_configs")},
                {'mode_configs': LaunchConfiguration("mode_configs")},
                {'load_configs': LaunchConfiguration("load_configs")},
            ]
        )
    ])