option(CAUSAL_SLAM_BUILD_BENCHMARKS
  "Build Causal-SLAM benchmark executables"
  ON
)

if(CAUSAL_SLAM_BUILD_BENCHMARKS)
  add_executable(point_time_extraction_benchmark
    tests/benchmarks/point_time_extraction_benchmark.cc
  )

  target_link_libraries(point_time_extraction_benchmark
      causal_slam_domain
  )

  install(TARGETS
    point_time_extraction_benchmark
    DESTINATION lib/${PROJECT_NAME}
  )
endif()
