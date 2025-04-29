import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Ruta al archivo de parámetros de Nav2
    nav2_params_path = os.path.join(
        get_package_share_directory("sancho_navigation"),
        "config",
        "nav2_params_ranger.yaml",
    )
    # Ruta al archivo de parámetros de depthimage_to_laserscan
    depthimage_to_laserscan_params = os.path.join(
        get_package_share_directory("sancho_navigation"),
        "config",
        "depthimage_to_laserscan_params.yaml",
    )

    return LaunchDescription(
        [
            # # Collision Monitor
            Node(
                 package="nav2_collision_monitor",
                 executable="collision_monitor",
                 name="collision_monitor",
                 prefix="xterm -hold -e",
                 emulate_tty=True,
                 parameters=[nav2_params_path],
                 remappings=[
                     ("cmd_vel_in", "cmd_vel_raw"),
                     ("cmd_vel_out", "cmd_vel"),
                 ],
             ),
            #  Nodo de mapa
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[nav2_params_path],
            ),
            # Nodo de localización (AMCL)
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[nav2_params_path],
            ),
            # Nodo del planner global
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[nav2_params_path],
            ),
            # Nodo del controlador de movimiento
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[nav2_params_path],
                remappings=[
                    ("cmd_vel", "cmd_vel")
                ],  # Importante redirigir salida al velocity smoother
            ),
            # Nodo de comportamiento basado en árboles de decisión (BT Navigator)
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[nav2_params_path],
            ),
            # Nodo de behaviors de Nav2
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[nav2_params_path],
            ),
            # Lifecycle Manager para control de nodos de Nav2
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
                parameters=[
                    {
                        "use_sim_time": False,
                        "autostart": True,
                        "node_names": [
                            "map_server",
                            "amcl",
                            "planner_server",
                            "controller_server",
                            "behavior_server",
                            "bt_navigator",
                            "collision_monitor",
                        ],
                    }
                ],
            ),
            Node(
                package="depthimage_to_laserscan",
                executable="depthimage_to_laserscan_node",  # Verifica el nombre del ejecutable según la documentación
                name="depthimage_to_laserscan",
                parameters=[depthimage_to_laserscan_params],    
                remappings=[
                    # Se remapean los nombres de los topics internos a los de tu sensor:
                    ("depth_camera_info", "/astra_camera/camera/depth/camera_info"),
                    ("depth", "/astra_camera/camera/depth/image_raw"),
                    ("scan", "/scan_camera"),
                ],
                output="screen"
            ),
        ]
    )
