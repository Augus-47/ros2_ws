from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_cmd_adapter = LaunchConfiguration("use_cmd_adapter")
    use_chassis_adapter = LaunchConfiguration("use_chassis_adapter")
    twist_to_vehiclecmd_params = LaunchConfiguration("twist_to_vehiclecmd_params")
    chassis_can_params = LaunchConfiguration("chassis_can_params")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_cmd_adapter",
            default_value="true",
            description="Whether to launch twist_to_vehiclecmd_node",
        ),

        DeclareLaunchArgument(
            "use_chassis_adapter",
            default_value="true",
            description="Whether to launch chassis_adapter_can_node",
        ),

        DeclareLaunchArgument(
            "twist_to_vehiclecmd_params",
            default_value=PathJoinSubstitution([
                FindPackageShare("legend_bringup"),
                "config",
                "twist_to_vehiclecmd.yaml",
            ]),
            description="Path to twist_to_vehiclecmd yaml file",
        ),

        DeclareLaunchArgument(
            "chassis_can_params",
            default_value=PathJoinSubstitution([
                FindPackageShare("legend_bringup"),
                "config",
                "chassis_can.yaml",
            ]),
            description="Path to chassis_can yaml file",
        ),

        Node(
            package="legend_cmd_adapter",
            executable="twist_to_vehiclecmd_node",
            name="twist_to_vehiclecmd_node",
            output="screen",
            parameters=[twist_to_vehiclecmd_params],
            condition=IfCondition(use_cmd_adapter),
        ),

        Node(
            package="legend_chassis_adapter_can",
            executable="chassis_adapter_can_node",
            name="chassis_adapter_can_node",
            output="screen",
            parameters=[chassis_can_params],
            condition=IfCondition(use_chassis_adapter),
        ),
    ])

