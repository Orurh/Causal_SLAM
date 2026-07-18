#pragma once

#include <iosfwd>

namespace causal_slam::apps::ros2 {

int RunSessionCli(int argc, char** argv, std::ostream& out, std::ostream& err);

}  // namespace causal_slam::apps::ros2
