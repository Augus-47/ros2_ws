import os

import launch
import launch.actions
import launch.events
import launch_ros
import launch_ros.actions
import launch_ros.events

from launch_ros.actions import LifecycleNode, Node
import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

from launch.actions import ExecuteProcess



def generate_launch_description():
    ld = launch.LaunchDescription()

    localization_param_dir = launch.substitutions.LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('lidar_localization_ros2'),
            'param',
            'localization.yaml'
        )
    )

    map_odom_repub = ExecuteProcess(
        cmd=[
            'python3', '/home/nvidia/auto_driving/map_odom_republisher.py',
            '--ros-args',
            '-p', 'pose_topic:=/pcl_pose',
            '-p', 'odom_topic:=/odom',
            '-p', 'publish_rate:=50.0',
            '-p', 'future_offset:=0.10',
            '-p', 'force_2d:=true',
            '-p', 'map_frame:=map',
            '-p', 'odom_frame:=odom',
            '-p', 'base_frame:=base_link',
        ],
        output='screen'
    )



    # ===== 1) 定位生命周期节点 =====
    lidar_localization = LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[localization_param_dir],
        # 你这里写一个就够：因为代码里订阅的是 "cloud"
        remappings=[
            ('cloud', '/rslidar_points'),
            ('/cloud', '/rslidar_points'),  # 保险：有的写成绝对名
        ],
        output='screen'
    )

    # ===== 2) 自动 configure =====
    configure_event = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # ===== 3) inactive -> activate =====
    activate_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg='[lidar_localization] -> activating'),
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    # ===== 4) 关键：保证 base_link <-> 雷达frame 的 TF 存在 =====
    # 如果你已经有 URDF + robot_state_publisher 在发，就可以把这个整段删掉/注释掉
    #
   


    # ===== add actions =====
    #ld.add_action(lidar_tf)
    ld.add_action(lidar_localization)
    ld.add_action(activate_handler)
    ld.add_action(configure_event)
    ld.add_action(map_odom_repub)

    # ⚠️ 建议先不要再启动 map_to_odom（否则和定位节点抢 TF）
    # ld.add_action(map_to_odom)

    return ld

