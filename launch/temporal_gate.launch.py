from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    imu_topic = LaunchConfiguration("imu_topic")
    lidar_topic = LaunchConfiguration("lidar_topic")
    checked_lidar_topic = LaunchConfiguration("checked_lidar_topic")

    map_update_allowed_topic = LaunchConfiguration("map_update_allowed_topic")
    temporal_health_topic = LaunchConfiguration("temporal_health_topic")
    map_update_reason_topic = LaunchConfiguration("map_update_reason_topic")
    fault_reasons_topic = LaunchConfiguration("fault_reasons_topic")
    map_update_decision_json_topic = LaunchConfiguration(
        "map_update_decision_json_topic"
    )
    diagnostics_topic = LaunchConfiguration("diagnostics_topic")
    runtime_profile = LaunchConfiguration("runtime_profile")
    html_report_path = LaunchConfiguration("html_report_path")

    lidar_qos_reliability = LaunchConfiguration("lidar_qos_reliability")
    lidar_qos_depth = LaunchConfiguration("lidar_qos_depth")
    checked_lidar_qos_reliability = LaunchConfiguration(
        "checked_lidar_qos_reliability"
    )
    checked_lidar_qos_depth = LaunchConfiguration("checked_lidar_qos_depth")

    lidar_gate_mode = LaunchConfiguration("lidar_gate_mode")
    tf_monitoring_enabled = LaunchConfiguration("tf_monitoring_enabled")
    tf_target_frame = LaunchConfiguration("tf_target_frame")
    tf_source_frame = LaunchConfiguration("tf_source_frame")

    expected_imu_period_ms = LaunchConfiguration("expected_imu_period_ms")
    imu_gap_threshold_ms = LaunchConfiguration("imu_gap_threshold_ms")
    max_missing_prefix_ms = LaunchConfiguration("max_missing_prefix_ms")
    max_missing_suffix_ms = LaunchConfiguration("max_missing_suffix_ms")
    max_internal_gap_ms = LaunchConfiguration("max_internal_gap_ms")

    return LaunchDescription([
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("lidar_topic", default_value="/points_raw"),
        DeclareLaunchArgument("checked_lidar_topic", default_value="/points_checked"),

        DeclareLaunchArgument(
            "map_update_allowed_topic",
            default_value="/causal_slam/map_update_allowed",
        ),
        DeclareLaunchArgument(
            "temporal_health_topic",
            default_value="/causal_slam/temporal_health",
        ),
        DeclareLaunchArgument(
            "map_update_reason_topic",
            default_value="/causal_slam/map_update_reason",
        ),
        DeclareLaunchArgument(
            "fault_reasons_topic",
            default_value="/causal_slam/fault_reasons",
        ),
        DeclareLaunchArgument(
            "map_update_decision_json_topic",
            default_value="/causal_slam/map_update_decision_json",
        ),
        DeclareLaunchArgument("diagnostics_topic", default_value="/diagnostics"),
        DeclareLaunchArgument(
            "runtime_profile",
            default_value="diagnostic",
            description="minimal, diagnostic, debug_report",
        ),
        DeclareLaunchArgument(
            "html_report_path",
            default_value="",
            description="empty disables periodic HTML report writing",
        ),

        DeclareLaunchArgument(
            "lidar_qos_reliability",
            default_value="best_effort",
            description="best_effort or reliable",
        ),
        DeclareLaunchArgument(
            "lidar_qos_depth",
            default_value="5",
            description="LiDAR subscription QoS depth",
        ),
        DeclareLaunchArgument(
            "checked_lidar_qos_reliability",
            default_value="best_effort",
            description="best_effort or reliable",
        ),
        DeclareLaunchArgument(
            "checked_lidar_qos_depth",
            default_value="5",
            description="checked LiDAR publisher QoS depth",
        ),

        DeclareLaunchArgument(
            "lidar_gate_mode",
            default_value="drop_degraded",
            description="observe, drop_invalid, drop_degraded, strict",
        ),

        DeclareLaunchArgument("tf_monitoring_enabled", default_value="true"),
        DeclareLaunchArgument("tf_target_frame", default_value="odom"),
        DeclareLaunchArgument("tf_source_frame", default_value="lidar"),

        DeclareLaunchArgument("expected_imu_period_ms", default_value="20.0"),
        DeclareLaunchArgument("imu_gap_threshold_ms", default_value="500.0"),
        DeclareLaunchArgument("max_missing_prefix_ms", default_value="30.0"),
        DeclareLaunchArgument("max_missing_suffix_ms", default_value="30.0"),
        DeclareLaunchArgument("max_internal_gap_ms", default_value="50.0"),

        Node(
            package="causal_slam",
            executable="temporal_monitor_node",
            name="temporal_monitor",
            output="screen",
            parameters=[{
                "imu_topic": imu_topic,
                "lidar_topic": lidar_topic,
                "checked_lidar_topic": checked_lidar_topic,
                "map_update_allowed_topic": map_update_allowed_topic,
                "temporal_health_topic": temporal_health_topic,
                "map_update_reason_topic": map_update_reason_topic,
                "fault_reasons_topic": fault_reasons_topic,
                "map_update_decision_json_topic": map_update_decision_json_topic,
                "lidar_gate_mode": lidar_gate_mode,
                "runtime_profile": runtime_profile,
                "html_report_path": html_report_path,
                "lidar_qos_reliability": lidar_qos_reliability,
                "lidar_qos_depth": lidar_qos_depth,
                "checked_lidar_qos_reliability": checked_lidar_qos_reliability,
                "checked_lidar_qos_depth": checked_lidar_qos_depth,
                "tf_monitoring_enabled": tf_monitoring_enabled,
                "tf_target_frame": tf_target_frame,
                "tf_source_frame": tf_source_frame,
                "expected_imu_period_ms": expected_imu_period_ms,
                "imu_gap_threshold_ms": imu_gap_threshold_ms,
                "max_missing_prefix_ms": max_missing_prefix_ms,
                "max_missing_suffix_ms": max_missing_suffix_ms,
                "max_internal_gap_ms": max_internal_gap_ms,
            }],
        ),

        Node(
            package="causal_slam",
            executable="temporal_status_bridge_node",
            name="temporal_status_bridge",
            output="screen",
            parameters=[{
                "map_update_allowed_topic": map_update_allowed_topic,
                "temporal_health_topic": temporal_health_topic,
                "map_update_reason_topic": map_update_reason_topic,
                "fault_reasons_topic": fault_reasons_topic,
                "map_update_decision_json_topic": map_update_decision_json_topic,
                "diagnostics_topic": diagnostics_topic,
            }],
        ),
    ])
