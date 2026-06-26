from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    fault_mode = LaunchConfiguration("fault_mode")
    tf_mode = LaunchConfiguration("tf_mode")

    use_fault_injector = PythonExpression([
        "'",
        fault_mode,
        "' != 'none'",
    ])

    use_static_tf = PythonExpression([
        "'",
        tf_mode,
        "' == 'healthy'",
    ])

    tf_monitoring_enabled = PythonExpression([
        "'",
        tf_mode,
        "' != 'off'",
    ])

    timestamp_shift_ms = PythonExpression([
        "'-150.0' if '",
        fault_mode,
        "' == 'imu_lag' else '0.0'",
    ])

    drop_every_n = PythonExpression([
        "'2' if '",
        fault_mode,
        "' == 'imu_drop' else '0'",
    ])

    imu_gap_threshold_ms = PythonExpression([
        "'30.0' if '",
        fault_mode,
        "' == 'imu_drop' else '500.0'",
    ])

    fake_imu_topic = PythonExpression([
        "'/causal_slam_demo/imu_raw' if '",
        fault_mode,
        "' != 'none' else '/causal_slam_demo/imu'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "fault_mode",
            default_value="none",
            description="Fault mode: none, imu_lag, imu_drop",
        ),
        DeclareLaunchArgument(
            "tf_mode",
            default_value="healthy",
            description="TF mode: off, healthy, missing",
        ),

        Node(
            package="causal_slam",
            executable="temporal_monitor_node",
            name="temporal_monitor",
            output="screen",
            parameters=[{
                "imu_topic": "/causal_slam_demo/imu",
                "lidar_topic": "/causal_slam_demo/points",
                "checked_lidar_topic": "/causal_slam_demo/points_checked",
                "map_update_allowed_topic": "/causal_slam_demo/map_update_allowed",
                "temporal_health_topic": "/causal_slam_demo/temporal_health",
                "map_update_decision_json_topic": "/causal_slam_demo/map_update_decision_json",
                "fault_reasons_topic": "/causal_slam_demo/fault_reasons",
                "map_update_reason_topic": "/causal_slam_demo/map_update_reason",
                "lidar_gate_mode": "drop_degraded",
                "imu_gap_threshold_ms": imu_gap_threshold_ms,
                "expected_imu_period_ms": 20.0,
                "max_missing_prefix_ms": 30.0,
                "max_missing_suffix_ms": 30.0,
                "max_internal_gap_ms": 50.0,
                "tf_monitoring_enabled": tf_monitoring_enabled,
                "tf_target_frame": "odom",
                "tf_source_frame": "lidar",
            }],
        ),

        Node(
            package="causal_slam",
            executable="imu_fault_injection_node",
            name="imu_fault_injection",
            output="screen",
            condition=IfCondition(use_fault_injector),
            parameters=[{
                "input_topic": "/causal_slam_demo/imu_raw",
                "output_topic": "/causal_slam_demo/imu",
                "timestamp_shift_ms": timestamp_shift_ms,
                "drop_every_n": drop_every_n,
            }],
        ),

        Node(
            package="causal_slam",
            executable="temporal_status_bridge_node",
            name="temporal_status_bridge",
            output="screen",
            parameters=[{
                "map_update_allowed_topic": "/causal_slam_demo/map_update_allowed",
                "temporal_health_topic": "/causal_slam_demo/temporal_health",
                "map_update_reason_topic": "/causal_slam_demo/map_update_reason",
                "fault_reasons_topic": "/causal_slam_demo/fault_reasons",
                "map_update_decision_json_topic": "/causal_slam_demo/map_update_decision_json",
                "diagnostics_topic": "/diagnostics",
            }],
        ),

        Node(
            package="causal_slam",
            executable="fake_imu_publisher_node",
            name="fake_imu",
            output="screen",
            parameters=[{
                "imu_topic": fake_imu_topic,
                "period_ms": 20.0,
                "timestamp_shift_ms": 0.0,
                "drop_every_n": 0,
            }],
        ),

        Node(
            package="causal_slam",
            executable="fake_lidar_publisher_node",
            name="fake_lidar",
            output="screen",
            parameters=[{
                "lidar_topic": "/causal_slam_demo/points",
                "period_ms": 100.0,
                "frame_id": "lidar",
                "scan_duration_ms": 100.0,
                "point_count": 32,
                "include_xyz_fields": True,
                "time_field_mode": "offset_time_uint32",
            }],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="healthy_static_tf",
            output="screen",
            condition=IfCondition(use_static_tf),
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "odom",
                "--child-frame-id", "lidar",
            ],
        ),
    ])
