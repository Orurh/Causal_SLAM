from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    default_config_file = PathJoinSubstitution([
        FindPackageShare("causal_slam"),
        "config",
        "temporal_gate.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="Path to Causal-SLAM temporal gate YAML config",
        ),

        Node(
            package="causal_slam",
            executable="temporal_monitor_node",
            name="temporal_monitor",
            output="screen",
            parameters=[config_file],
        ),

        Node(
            package="causal_slam",
            executable="temporal_status_bridge_node",
            name="temporal_status_bridge",
            output="screen",
            parameters=[config_file],
        ),
    ])
