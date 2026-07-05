#pragma once

#include <initializer_list>
#include <string>
#include <string_view>

#include "presentation/report/report_document.h"

namespace causal_slam::render {

class ReportDocumentHtmlRenderer final {
 public:
  [[nodiscard]] std::string RenderBody(const causal_slam::report::ReportDocument& document) const;
  [[nodiscard]] std::string RenderPage(const causal_slam::report::ReportDocument& document) const;
  [[nodiscard]] std::string RenderPage(std::string_view page_title,
                                       std::initializer_list<const causal_slam::report::ReportDocument*> documents) const;
};

}  // namespace causal_slam::render