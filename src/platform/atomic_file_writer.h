#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace causal_slam::platform {

struct WriteTextFileResult {
  bool ok{false};
  std::string error;
};

[[nodiscard]] WriteTextFileResult WriteTextFileAtomically(const std::filesystem::path& output_path, std::string_view content);

}  // namespace causal_slam::platform
