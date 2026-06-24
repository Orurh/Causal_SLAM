if(NOT BUILD_TESTING)
  return()
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(ament_cmake_gtest REQUIRED)

function(causal_slam_add_gtests_from_dir test_dir link_target)
  set(multi_value_args AMENT_DEPS)
  cmake_parse_arguments(ARG "" "" "${multi_value_args}" ${ARGN})

  file(GLOB test_sources
    CONFIGURE_DEPENDS
      "${CMAKE_CURRENT_SOURCE_DIR}/${test_dir}/*_test.cc"
  )

  foreach(test_source IN LISTS test_sources)
    get_filename_component(test_target "${test_source}" NAME_WE)

    ament_add_gtest(${test_target}
      ${test_source}
    )

    if(TARGET ${test_target})
      target_link_libraries(${test_target}
          ${link_target}
      )

      if(ARG_AMENT_DEPS)
        ament_target_dependencies(${test_target}
          ${ARG_AMENT_DEPS}
        )
      endif()
    endif()
  endforeach()
endfunction()

causal_slam_add_gtests_from_dir(tests/telemetry causal_slam_domain)
causal_slam_add_gtests_from_dir(tests/transform causal_slam_domain)
causal_slam_add_gtests_from_dir(tests/lidar causal_slam_domain)
causal_slam_add_gtests_from_dir(tests/pointcloud causal_slam_domain)
causal_slam_add_gtests_from_dir(tests/coverage causal_slam_domain)
causal_slam_add_gtests_from_dir(tests/diagnostics causal_slam_domain)
causal_slam_add_gtests_from_dir(tests/statistics causal_slam_domain)
causal_slam_add_gtests_from_dir(tests/policy causal_slam_domain)

causal_slam_add_gtests_from_dir(tests/config causal_slam_application)
causal_slam_add_gtests_from_dir(tests/pipeline causal_slam_application)

causal_slam_add_gtests_from_dir(tests/report causal_slam_presentation)
causal_slam_add_gtests_from_dir(tests/render causal_slam_presentation)

causal_slam_add_gtests_from_dir(tests/platform causal_slam_platform)

causal_slam_add_gtests_from_dir(
  tests/ros_adapters
  causal_slam_ros_adapters
  AMENT_DEPS
    rclcpp
    sensor_msgs
    builtin_interfaces
    geometry_msgs
    tf2
    tf2_ros
)

add_test(
  NAME config_parameter_drift_test
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_config_params.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_test(
  NAME architecture_dependency_test
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_arch_deps.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_test(
  NAME ros1_compatibility_audit_test
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/audit_ros1_compatibility.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_test(
  NAME ros1_adapter_audit_test
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/audit_ros1_adapter.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
