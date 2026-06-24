#include "platform/atomic_file_writer.h"

#include <fstream>
#include <system_error>

namespace causal_slam::platform {
namespace {

std::filesystem::path MakeTemporaryPath(
    const std::filesystem::path& output_path) {
  auto temporary_path = output_path;
  temporary_path += ".tmp";
  return temporary_path;
}

}  // namespace

WriteTextFileResult WriteTextFileAtomically(
    const std::filesystem::path& output_path,
    std::string_view content) {
  if (output_path.empty()) {
    return WriteTextFileResult{
        .ok = false,
        .error = "output_path_is_empty",
    };
  }

  std::error_code error_code;

  const auto parent_path = output_path.parent_path();
  if (!parent_path.empty()) {
    std::filesystem::create_directories(parent_path, error_code);
    if (error_code) {
      return WriteTextFileResult{
          .ok = false,
          .error = "failed_to_create_parent_directory: " +
                   error_code.message(),
      };
    }
  }

  const auto temporary_path = MakeTemporaryPath(output_path);

  {
    std::ofstream file(temporary_path, std::ios::binary | std::ios::trunc);
    if (!file.is_open()) {
      return WriteTextFileResult{
          .ok = false,
          .error = "failed_to_open_temporary_file",
      };
    }

    file.write(content.data(), static_cast<std::streamsize>(content.size()));

    if (!file.good()) {
      return WriteTextFileResult{
          .ok = false,
          .error = "failed_to_write_temporary_file",
      };
    }
  }

  std::filesystem::rename(temporary_path, output_path, error_code);
  if (error_code) {
    std::filesystem::remove(temporary_path);
    return WriteTextFileResult{
        .ok = false,
        .error = "failed_to_replace_output_file: " + error_code.message(),
    };
  }

  return WriteTextFileResult{.ok = true, .error = ""};
}

}  // namespace causal_slam::platform
