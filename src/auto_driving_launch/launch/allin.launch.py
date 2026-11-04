from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.parameter_descriptions import Parameter

def generate_launch_description():
    ld = LaunchDescription()
    
    # 1. 启动雷达驱动
    rslidar_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'rslidar_sdk', 'start.py'],
        output='screen'
    )
    ld.add_action(rslidar_launch)
    
    # 新增: 启动 pointcloud_to_laserscan 节点
    pointcloud_to_laserscan_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node',
            '--ros-args',
            '-r', 'cloud_in:=/rslidar_points',  # 重映射点云输入话题
            '-r', 'scan:=/scan',
            '-p', 'queue_size:=1',    
            '-p', 'transform_tolerance:=0.0'
        ],
        output='screen'
    )
    ld.add_action(TimerAction(period=1.0, actions=[pointcloud_to_laserscan_node]))
    
    # 2. 启动激光雷达定位
    localization_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'lidar_localization_ros2', 'lidar_localization.launch.py'],
        output='screen'
    )
    ld.add_action(localization_launch)
    
    # 3. 启动地图服务器 - 修复：添加namespace参数
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',  # 添加这个必需的参数
        output='screen',
        parameters=[{'yaml_filename': '/home/nvidia/auto_driving/map.yaml'}]
    )
    ld.add_action(map_server_node)
    
    # 4. 配置并激活地图服务器（延迟执行）
    ld.add_action(TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
            output='screen'
        )]
    ))
    
    ld.add_action(TimerAction(
        period=8.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
            output='screen'
        )]
    ))
    
    # 5. 启动静态TF
    #static_tf_launch = ExecuteProcess(
    #    cmd=['ros2', 'launch', 'static_tf.launch.py'],
    #    output='screen'
    #)
    #ld.add_action(TimerAction(period=3.0, actions=[static_tf_launch]))
    
    # 7. 启动车辆控制接口
    car_cmd_node = ExecuteProcess(
        cmd=['ros2', 'run', 'car_cmd_interface', 'car_cmd_interface_node'],
        output='screen'
    )
    ld.add_action(TimerAction(period=14.0, actions=[car_cmd_node]))
    
    
    
    return ld
