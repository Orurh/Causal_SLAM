function(causal_slam_target_include_src target_name)
  target_include_directories(${target_name}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
endfunction()

function(causal_slam_install_ros2_executable target_name)
  install(TARGETS
    ${target_name}
    DESTINATION lib/${PROJECT_NAME}
  )
endfunction()
