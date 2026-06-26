add_executable(temporal_monitor_node
  src/apps/ros2/temporal_monitor_main.cc
  src/apps/ros2/temporal_monitor_node.cc
  src/apps/ros2/temporal_monitor_node_parameters.cc
)

target_link_libraries(temporal_monitor_node
    causal_slam_application
    causal_slam_presentation
    causal_slam_platform
    causal_slam_ros_adapters)

ament_target_dependencies(temporal_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
  builtin_interfaces
  geometry_msgs
  tf2
  tf2_ros
)

causal_slam_target_include_src(temporal_monitor_node)
causal_slam_install_ros2_executable(temporal_monitor_node)

add_executable(temporal_status_bridge_node
  src/apps/ros2/temporal_status_bridge_node.cc
)

ament_target_dependencies(temporal_status_bridge_node
  rclcpp
  std_msgs
  diagnostic_msgs
)

causal_slam_target_include_src(temporal_status_bridge_node)
causal_slam_install_ros2_executable(temporal_status_bridge_node)

# Developer/demo ROS2 tools.
add_executable(fake_imu_publisher_node
  tools/ros2/fake_imu_publisher_node.cc
)

ament_target_dependencies(fake_imu_publisher_node
  rclcpp
  sensor_msgs
)

causal_slam_target_include_src(fake_imu_publisher_node)
causal_slam_install_ros2_executable(fake_imu_publisher_node)

add_executable(fake_lidar_publisher_node
  tools/ros2/fake_lidar_publisher_node.cc
)

ament_target_dependencies(fake_lidar_publisher_node
  rclcpp
  sensor_msgs
)

causal_slam_target_include_src(fake_lidar_publisher_node)
causal_slam_install_ros2_executable(fake_lidar_publisher_node)

add_executable(imu_fault_injection_node
  tools/ros2/imu_fault_injection_node.cc
)

ament_target_dependencies(imu_fault_injection_node
  rclcpp
  sensor_msgs
)

causal_slam_target_include_src(imu_fault_injection_node)
causal_slam_install_ros2_executable(imu_fault_injection_node)

add_executable(point_cloud_rate_probe_node
  tools/ros2/point_cloud_rate_probe_node.cc
)

ament_target_dependencies(point_cloud_rate_probe_node
  rclcpp
  sensor_msgs
)

causal_slam_target_include_src(point_cloud_rate_probe_node)
causal_slam_install_ros2_executable(point_cloud_rate_probe_node)
