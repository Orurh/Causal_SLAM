#pragma once

#include <cstdint>

#include "domain/telemetry/temporal_health.h"

namespace causal_slam::policy {

enum class MapUpdateDecisionReason : std::uint8_t {
  kTemporalHealthOk,
  kTemporalHealthWarning,
  kTemporalHealthDegraded,
  kTemporalHealthInvalid,
};

struct MapUpdateDecision {
  bool map_update_allowed{true};
  MapUpdateDecisionReason reason{MapUpdateDecisionReason::kTemporalHealthOk};
};

[[nodiscard]] inline const char* ToString(MapUpdateDecisionReason reason) {
  switch (reason) {
    case MapUpdateDecisionReason::kTemporalHealthOk:
      return "temporal_health_ok";
    case MapUpdateDecisionReason::kTemporalHealthWarning:
      return "temporal_health_warning";
    case MapUpdateDecisionReason::kTemporalHealthDegraded:
      return "temporal_health_degraded";
    case MapUpdateDecisionReason::kTemporalHealthInvalid:
      return "temporal_health_invalid";
  }

  return "unknown";
}

[[nodiscard]] inline MapUpdateDecision DecideMapUpdate(causal_slam::telemetry::TemporalHealthStatus status, bool has_hard_fusion_blocker) {
  using causal_slam::telemetry::TemporalHealthStatus;

  switch (status) {
    case TemporalHealthStatus::kOk:
      return MapUpdateDecision{
          .map_update_allowed = true,
          .reason = MapUpdateDecisionReason::kTemporalHealthOk,
      };
    case TemporalHealthStatus::kWarning:
      return MapUpdateDecision{
          .map_update_allowed = true,
          .reason = MapUpdateDecisionReason::kTemporalHealthWarning,
      };
    case TemporalHealthStatus::kDegraded:
      return MapUpdateDecision{
          .map_update_allowed = !has_hard_fusion_blocker,
          .reason = MapUpdateDecisionReason::kTemporalHealthDegraded,
      };
    case TemporalHealthStatus::kInvalid:
      return MapUpdateDecision{
          .map_update_allowed = false,
          .reason = MapUpdateDecisionReason::kTemporalHealthInvalid,
      };
  }

  return MapUpdateDecision{
      .map_update_allowed = false,
      .reason = MapUpdateDecisionReason::kTemporalHealthInvalid,
  };
}

[[nodiscard]] inline MapUpdateDecision DecideMapUpdate(causal_slam::telemetry::TemporalHealthStatus status) {
  using causal_slam::telemetry::TemporalHealthStatus;

  return DecideMapUpdate(status, status == TemporalHealthStatus::kDegraded);
}

}  // namespace causal_slam::policy
