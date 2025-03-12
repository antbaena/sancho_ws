from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value=''),
        DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model')),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('use_world_frame', default_value='true'),
        DeclareLaunchArgument('external_urdf_loc', default_value=''),
        DeclareLaunchArgument('use_joint_pub', default_value='false'),
        DeclareLaunchArgument('use_joint_pub_gui', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('load_gazebo_configs', default_value='false'),
        DeclareLaunchArgument('rvizconfig', default_value=[get_package_share_directory('interbotix_xsturret_descriptions'), '/rviz/xsturret_description.rviz']),
        DeclareLaunchArgument('model', default_value=[get_package_share_directory('interbotix_xsturret_descriptions'), '/urdf/', LaunchConfiguration('robot_model'),'.urdf.xacro robot_name:=', LaunchConfiguration('robot_name'),' base_link_frame:=', LaunchConfiguration('base_link_frame'), ' use_world_frame:=', LaunchConfiguration('use_world_frame'), ' external_urdf_loc:=', LaunchConfiguration('external_urdf_loc'), ' load_gazebo_configs:=', LaunchConfiguration('load_gazebo_configs')]),

        LogInfo(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            args=['Launching RViz with config: ', LaunchConfiguration('rvizconfig')]
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('use_joint_pub')),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=LaunchConfiguration('robot_name'),
            name='joint_state_publisher'
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('use_joint_pub_gui')),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace=LaunchConfiguration('robot_name'),
            name='joint_state_publisher_gui'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('robot_name'),
            name='robot_state_publisher'
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            namespace=LaunchConfiguration('robot_name'),
            name='rviz'
        ),
    ])
