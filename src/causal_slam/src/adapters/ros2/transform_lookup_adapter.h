#pragma once

#include <cstdint>
#include <string>

#include <tf2_ros/buffer.h>

#include "domain/sensors/transform/transform_lookup_observation.h"

namespace causal_slam::ros_adapters {

struct TransformLookupRequest {
  std::string target_frame;
  std::string source_frame;

  std::int64_t requested_stamp_ns{0};
  std::int64_t receive_time_ns{0};
};

[[nodiscard]] causal_slam::transform::TransformLookupObservation LookupTransform(
    tf2_ros::Buffer& tf_buffer,
    const TransformLookupRequest& request);

}  // namespace causal_slam::ros_adapters
