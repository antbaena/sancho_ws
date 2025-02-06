import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Ruta al archivo de parámetros de Nav2
    nav2_params_path = os.path.join(
        get_package_share_directory('ranger_nav2'),
        'config',
        'nav2_params_ranger.yaml'
    )

    # Ruta al paquete de bringup del ranger
    ranger_bringup_path = get_package_share_directory('ranger_bringup')

    # Ruta al modelo URDF del robot
    urdf_file_path = os.path.join(
        get_package_share_directory('ranger_nav2'),
        'config',
        'ranger.urdf'
    )
    
    hokuyo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('urg_node2'), 'launch'),
         '/urg_node2_2lidar.launch.py'])
      )

    return LaunchDescription([
        # Publicar estado del robot desde URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file_path).read()}]
        ),

        # Publicar estado de las articulaciones (si el robot tiene articulaciones móviles)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Incluir el launch de la base (driver, etc.)
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(ranger_bringup_path, 'launch', 'ranger_mini_v2.launch.xml'))
        ),

        # Nodos básicos de navegación
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params_path]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_path]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_path]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_path]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_path]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }]
        ),

        # Nodo del LiDAR 
        hokuyo_launch,

        # Nodo de behaviors de Nav2 (si lo usas)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_path]
        ),
    ])
