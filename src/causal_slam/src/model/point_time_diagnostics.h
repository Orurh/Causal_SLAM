#pragma once

#include <string>

namespace causal_slam::model {

struct PointTimeDiagnostics {
  bool has_time_candidate{false};
  bool has_supported_time_field{false};

  std::string field_name;
  std::string field_datatype;
  std::string field_role;

  std::string inspection_reason;

  bool extraction_attempted{false};
  bool extraction_used{false};
  std::string extraction_reason;
  std::string extraction_unit;
};

}  // namespace causal_slam::model
