import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ranger_bringup_path = get_package_share_directory('ranger_bringup')

    # Cargar URDF desde sancho_description
    urdf_file_path = os.path.join(
        get_package_share_directory('sancho_description'),
        'urdf',
        'sancho_ranger.urdf'
    )

    # Lanzar LiDAR Hokuyo
    hokuyo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('urg_node2'), 'launch', 'urg_node2_2lidar.launch.py')
        )
    )

    # Lanzar cámara Orbbec Gemini 330 en namespace independiente
    orbbec_camera_launch = GroupAction([
        PushRosNamespace('gemini_camera'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'gemini_330_series.launch.py')
            ),
            launch_arguments={
                'camera_name': 'gemini_cam',
                'serial_number': 'AARX3410080',
                'usb_port': '2-1',
                'device_num': '1',
                'sync_mode': 'standalone'
            }.items()
        )
    ])

    # Lanzar cámara Astra en namespace independiente
    astra_camera_launch = GroupAction([
        PushRosNamespace('astra_camera'),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory('astra_camera'), 'launch', 'astra.launch.xml')
            ),
            launch_arguments={
                'serial_number': "'20070830098'",
                'camera_name': 'camera',
                'color_fps': '15',
                'depth_fps': '15',
                'ir_fps': '15'

                
            }.items()
        )
    ])

    ranger_launch = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(os.path.join(ranger_bringup_path, 'launch', 'ranger_mini_v2.launch.xml')),
            launch_arguments={
        'base_frame': 'base_footprint',
        }.items()
        )

    return LaunchDescription([
        # robot_state_publisher (URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file_path).read()}]
        ),

        # joint_state_publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),
                # Nodo para fusionar los dos LaserScan en uno solo (/scan)
         Node(
            package='laser_scan_merger',
            executable='laser_scan_merger',
            name='laser_scan_merger',
            output='screen',
            prefix='xterm -hold -e',
            parameters=[{
                'scanTopic1': '/scan_1st',
                'scanTopic2': '/scan_2nd',
                'target_frame': 'base_link',
                'laser1_frame': 'laser_front',
                'laser2_frame': 'laser_back',
                'point_cloud_topic': 'scan_merged',
                'pointCloutFrameId': 'base_link',
                'pointCloudTopic':  '/laser_cloud',
            }]
        ),

        # Nodo para convertir PointCloud2 a LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='cloud_to_scan',
            output='screen',
            prefix='xterm -hold -e',
            remappings=[
                ('cloud_in', '/laser_cloud'),
                ('scan', '/scan_merged')
            ],
            parameters=[{
                'target_frame': 'base_link', # <- tu frame deseado
                'transform_tolerance': 0.01,
                'min_height': -1.0,
                'max_height': 1.0,
                'angle_min': -3.14,
                'angle_max': 3.14,
                'angle_increment': 0.008,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
        ),



        # Launch de la base móvil Ranger
        ranger_launch,

        # Lanzamiento de sensores
        hokuyo_launch,
        astra_camera_launch
    ])