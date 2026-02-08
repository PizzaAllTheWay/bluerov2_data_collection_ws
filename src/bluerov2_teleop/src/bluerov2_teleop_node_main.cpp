#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "bluerov2_teleop/bluerov2_teleop_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // rclcpp::spin(std::make_unique<bluerov2_teleop::BlueROVTeleOpNode>(rclcpp::NodeOptions()));
  rclcpp::spin(std::make_shared<bluerov2_teleop::BlueROVTeleOpNode>(rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}