// Copyright <2020> [Copyright rossihwang]

#include "fitkits_imu/hi21x.h"
#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<fitkits_imu::Hi21x>("hi21x_node", rclcpp::NodeOptions());

  rclcpp::spin(node);
  rclcpp::shutdown();

  node = nullptr;

  return 0;
}