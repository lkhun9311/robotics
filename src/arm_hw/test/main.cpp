// SPDX-License-Identifier: Apache-2.0
// The hardware components create their own rclcpp node: rclcpp is initialized around the tests.
#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
