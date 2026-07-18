#include "apps/ros2/session_app.h"

#include <iostream>

int main(int argc, char** argv) {
  return causal_slam::apps::ros2::RunSessionCli(argc, argv, std::cout, std::cerr);
}
