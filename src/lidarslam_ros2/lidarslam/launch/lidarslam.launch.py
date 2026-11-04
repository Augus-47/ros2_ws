import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'param',
            'lidarslam.yaml'))
    
    rviz_param_dir = launch.substitutions.LaunchConfiguration(
        'rviz_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'rviz',
            'mapping.rviz'))

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=["/home/nvidia/auto_driving/src/localization/lidarslam_ros2/lidarslam/param/lidarslam.yaml"],
        # remappings=[('/input_cloud','/velodyne_points')],  # 修改前
        remappings=[('input_cloud','/rslidar_points')],
        output='screen'
        )

    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # arguments=['0','0','0','0','0','0','1','base_link','velodyne_top']  #修改之前的
        arguments=[
            '0','0','1.0',
            '0','0','0',
            'base_link','rslidar']   #修改后
        )


    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        # parameters=[main_param_dir],    修改之前      
        parameters=["/home/nvidia/auto_driving/src/localization/lidarslam_ros2/lidarslam/param/lidarslam.yaml"],
        output='screen'
        )
    
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_param_dir]
        )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'main_param_dir',
            #default_value=main_param_dir,
            default_value="/home/nvidia/auto_driving/src/localization/lidarslam_ros2/lidarslam/param/lidarslam.yaml",
            description='Full path to main parameter file to load'),
        mapping,
        tf,
        graphbasedslam,
        rviz,
            ])
