#include "platform/atomic_file_writer.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace causal_slam::platform {
namespace {

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  std::ostringstream out;
  out << file.rdbuf();
  return out.str();
}

std::filesystem::path MakeTempDir() {
  const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();

  return std::filesystem::temp_directory_path() / ("causal_slam_atomic_writer_test_" + std::to_string(suffix));
}

TEST(AtomicFileWriterTest, WritesNewFile) {
  const auto dir = MakeTempDir();
  const auto path = dir / "report.html";

  const auto result = WriteTextFileAtomically(path, "<html>ok</html>");

  EXPECT_TRUE(result.ok) << result.error;
  EXPECT_EQ(ReadFile(path), "<html>ok</html>");

  std::filesystem::remove_all(dir);
}

TEST(AtomicFileWriterTest, ReplacesExistingFile) {
  const auto dir = MakeTempDir();
  const auto path = dir / "report.html";

  ASSERT_TRUE(WriteTextFileAtomically(path, "old").ok);
  const auto result = WriteTextFileAtomically(path, "new");

  EXPECT_TRUE(result.ok) << result.error;
  EXPECT_EQ(ReadFile(path), "new");

  std::filesystem::remove_all(dir);
}

TEST(AtomicFileWriterTest, RejectsEmptyPath) {
  const auto result = WriteTextFileAtomically("", "content");

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, "output_path_is_empty");
}

}  // namespace
}  // namespace causal_slam::platform
