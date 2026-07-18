#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "domain/sensors/pointcloud/point_cloud2_datatype.h"
#include "domain/sensors/pointcloud/point_cloud2_time_field_extractor.h"

namespace {

using Clock = std::chrono::steady_clock;

constexpr std::uint32_t kPointStep = 16;
constexpr std::uint32_t kOffsetTimeFieldOffset = 12;
constexpr std::int64_t kHeaderStampNs = 1'000'000'000LL;
constexpr std::uint32_t kScanDurationNs = 100'000'000U;

std::vector<std::uint8_t> MakeCloudData(std::uint32_t point_count) {
  std::vector<std::uint8_t> data(static_cast<std::size_t>(point_count) * kPointStep, 0U);

  for (std::uint32_t i = 0; i < point_count; ++i) {
    const std::uint32_t offset_ns =
        point_count <= 1
            ? 0U
            : static_cast<std::uint32_t>((static_cast<std::uint64_t>(i) * kScanDurationNs) / static_cast<std::uint64_t>(point_count - 1));

    const std::size_t byte_offset = (static_cast<std::size_t>(i) * kPointStep) + kOffsetTimeFieldOffset;

    std::memcpy(data.data() + byte_offset, &offset_ns, sizeof(offset_ns));
  }

  return data;
}

void RunCase(std::uint32_t point_count, int iterations) {
  const auto data = MakeCloudData(point_count);

  causal_slam::pointcloud::PointCloud2CloudView cloud;
  cloud.header_stamp_ns = kHeaderStampNs;
  cloud.width = point_count;
  cloud.height = 1;
  cloud.point_step = kPointStep;
  cloud.data = data.data();
  cloud.data_size = data.size();

  causal_slam::pointcloud::PointCloud2FieldInfo time_field;
  time_field.name = "offset_time";
  time_field.offset = kOffsetTimeFieldOffset;
  time_field.datatype =
      causal_slam::pointcloud::kPointCloud2Uint32;
  time_field.count = 1;
  time_field.time_role =
      causal_slam::pointcloud::PointCloud2TimeFieldRole::
          kPointOffsetTime;

  const causal_slam::pointcloud::PointCloud2TimeFieldExtractor extractor;

  // Warmup.
  const auto warmup = extractor.Extract(cloud, time_field);
  if (!warmup.has_scan_window) {
    throw std::runtime_error("warmup extraction failed: " + warmup.reason);
  }

  std::vector<double> durations_us;
  durations_us.reserve(static_cast<std::size_t>(iterations));

  for (int i = 0; i < iterations; ++i) {
    const auto started = Clock::now();
    const auto result = extractor.Extract(cloud, time_field);
    const auto finished = Clock::now();

    if (!result.has_scan_window) {
      throw std::runtime_error("extraction failed: " + result.reason);
    }

    if (result.point_count_used != point_count) {
      throw std::runtime_error("unexpected point_count_used");
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(finished - started);

    durations_us.push_back(elapsed.count());
  }

  std::sort(durations_us.begin(), durations_us.end());

  const double total_us = std::accumulate(durations_us.begin(), durations_us.end(), 0.0);
  const double avg_us = total_us / static_cast<double>(durations_us.size());
  const double p50_us = durations_us[durations_us.size() / 2];
  const double p95_us = durations_us[static_cast<std::size_t>(static_cast<double>(durations_us.size() - 1) * 0.95)];
  const double max_us = durations_us.back();

  std::cout << std::fixed << std::setprecision(2) << "point_count=" << point_count << " iterations=" << iterations << " avg_us=" << avg_us
            << " p50_us=" << p50_us << " p95_us=" << p95_us << " max_us=" << max_us << " avg_ms=" << avg_us / 1000.0 << '\n';
}

bool StressBenchmarkEnabled() {
  const char* value = std::getenv("CAUSAL_SLAM_BENCHMARK_STRESS");
  if (value == nullptr) {
    return false;
  }

  const std::string flag{value};
  return flag == "1" || flag == "true" || flag == "TRUE" || flag == "on";
}

}  // namespace

int main() {
  try {
    RunCase(1'000, 200);
    RunCase(100'000, 50);
    RunCase(1'000'000, 10);

    if (StressBenchmarkEnabled()) {
      RunCase(5'000'000, 5);
      RunCase(10'000'000, 3);
    }
  } catch (const std::exception& error) {
    std::cerr << "benchmark failed: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
