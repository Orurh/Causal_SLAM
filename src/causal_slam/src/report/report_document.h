#pragma once

#include <string>
#include <vector>

namespace causal_slam::report {

struct ReportMetric {
  std::string name;
  std::string value;
};

struct ReportRow {
  std::string label;
  std::string status;

  std::vector<ReportMetric> metrics;

  std::string reason;
  std::string detail;

  std::string explanation;
  std::string evidence;
  std::string suggested_action;
};

struct ReportSection {
  std::string id;
  std::string title;
  std::string empty_message{"none"};

  std::vector<ReportMetric> metrics;
  std::vector<ReportRow> rows;
};

struct ReportDocument {
  std::string title;
  std::vector<ReportSection> sections;
};

}  // namespace causal_slam::report
