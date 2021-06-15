#include "sdkeli_ls_udp/sdkeli_ls2027de.h"
#include <iostream>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<sdkeli_ls_udp::LS2027DE>("sdkeli_ls2027de_node", rclcpp::NodeOptions());
  if (node->init()) {
    rclcpp::spin(node);
    rclcpp::shutdown();
  } else {
    std::cout << "sdkeli_ls2027de_node init failed";
    return 1;
  }
  node = nullptr;

  return 0;
}
