#include "ros_adapters/transform_lookup_adapter.h"

#include <rcl/time.h>
#include <rclcpp/time.hpp>
#include <tf2/exceptions.h>

namespace causal_slam::ros_adapters {
namespace {

causal_slam::transform::TransformLookupObservation MakeBaseObservation(
    const TransformLookupRequest& request) {
  causal_slam::transform::TransformLookupObservation observation;
  observation.target_frame = request.target_frame;
  observation.source_frame = request.source_frame;
  observation.requested_stamp_ns = request.requested_stamp_ns;
  observation.receive_time_ns = request.receive_time_ns;
  return observation;
}

}  // namespace

causal_slam::transform::TransformLookupObservation LookupTransform(
    tf2_ros::Buffer& tf_buffer,
    const TransformLookupRequest& request) {
  auto observation = MakeBaseObservation(request);

  if (request.target_frame.empty()) {
    observation.lookup_success = false;
    observation.extrapolation_required = false;
    observation.failure_reason = "empty_target_frame";
    return observation;
  }

  if (request.source_frame.empty()) {
    observation.lookup_success = false;
    observation.extrapolation_required = false;
    observation.failure_reason = "empty_source_frame";
    return observation;
  }

  try {
    const auto requested_time =
        rclcpp::Time(request.requested_stamp_ns, RCL_ROS_TIME);

    const auto transform =
        tf_buffer.lookupTransform(
            request.target_frame,
            request.source_frame,
            requested_time);

    observation.lookup_success = true;
    observation.extrapolation_required = false;
    observation.transform_stamp_ns =
        rclcpp::Time(transform.header.stamp).nanoseconds();
    observation.failure_reason = "";
    return observation;
  } catch (const tf2::ExtrapolationException& error) {
    observation.lookup_success = false;
    observation.extrapolation_required = true;
    observation.failure_reason = error.what();
    return observation;
  } catch (const tf2::TransformException& error) {
    observation.lookup_success = false;
    observation.extrapolation_required = false;
    observation.failure_reason = error.what();
    return observation;
  }
}

}  // namespace causal_slam::ros_adapters
