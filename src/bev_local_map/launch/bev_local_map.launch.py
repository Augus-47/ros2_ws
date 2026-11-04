from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('bev_local_map')
    params_file = os.path.join(pkg_share, 'config', 'bev_params.yaml')

    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/rslidar_points'
    )

    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='base_link'
    )

    bev_node = Node(
        package='bev_local_map',
        executable='bev_local_map_node',
        name='bev_local_map_node',
        output='screen',
        parameters=[
            params_file,
            {
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'target_frame': LaunchConfiguration('target_frame'),
            }
        ]
    )

    return LaunchDescription([
        cloud_topic_arg,
        target_frame_arg,
        bev_node
    ])

