import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    start_patrol = LaunchConfiguration('start_patrol')
    patrol_params = LaunchConfiguration('patrol_params')

    # 包路径
    autopartol_share = get_package_share_directory('autopartol_robot')
    default_patrol_params = os.path.join(autopartol_share, 'config', 'partol_config.yaml')

    # 把 nav2_params.yaml 和 rviz 配置复制到配置的包下
    default_nav2_params = os.path.join(autopartol_share, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(autopartol_share, 'rviz', 'nav2_default_view.rviz')



    # 创建新的auto_driving_launch功能包
    static_tf_launch = os.path.join(get_package_share_directory('auto_driving_launch'), 'launch', 'static_sensors_tf.launch.py')
    allin_launch = os.path.join(get_package_share_directory('auto_driving_launch'), 'launch', 'allin.launch.py')

    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')


    # RViz
    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # 巡检节点
    patrol_node = Node(
        condition=IfCondition(start_patrol),
        package='autopartol_robot',
        executable='partol_node',
        name='partol_node',
        output='screen',
        parameters=[patrol_params]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # DeclareLaunchArgument('map', default_value='/home/nvidia/ros2_ws/map.yaml'),
        DeclareLaunchArgument('map', default_value='/home/nvidia/auto_driving/map.yaml'),
        DeclareLaunchArgument('nav2_params', default_value=default_nav2_params),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        DeclareLaunchArgument('start_patrol', default_value='true'),
        DeclareLaunchArgument('patrol_params', default_value=default_patrol_params),


        # 3) EKF / 静态TF / allin
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_tf_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(allin_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # 4) Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml,
                'params_file': nav2_params,
            }.items()
        ),

        # 5) RViz
        rviz_node,

        # 6) Patrol
        patrol_node,
    ])

