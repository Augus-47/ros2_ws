from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('auto_driving_launch')  # 你的包名
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_localization.yaml')

    # 默认 false：真实机器人用系统 ROS 时间
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (/clock) time if true'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[
                ekf_config,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('odometry/filtered', 'odom')
            ]
        )
    ])

