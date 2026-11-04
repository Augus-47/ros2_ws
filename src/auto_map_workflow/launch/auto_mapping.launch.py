from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    lidarslam_launch = os.path.join(
        get_package_share_directory('lidarslam'),
        'launch',
        'lidarslam.launch.py'
    )

    config_file = os.path.join(
        get_package_share_directory('auto_map_workflow'),
        'config',
        'auto_mapping.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidarslam_launch)
        ),

        Node(
            package='auto_map_workflow',
            executable='map_workflow',
            name='map_workflow',
            output='screen',
            parameters=[config_file]
        ),

        Node(
            package='auto_map_workflow',
            executable='map_pgm_to_png_watcher',
            name='map_pgm_to_png_watcher',
            output='screen',
            parameters=[
                {'watch_dir': '/home/nvidia/ros2_ws/maps'},
                {'check_interval': 1.0},
                {'stable_checks': 3}
            ]
        )
    ])

