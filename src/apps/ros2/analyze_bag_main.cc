#include <iostream>

#include "apps/ros2/analyze_bag_app.h"

int main(int argc, char** argv) {
  return causal_slam::apps::ros2::RunAnalyzeBagCli(argc, argv, std::cout, std::cerr);
}
