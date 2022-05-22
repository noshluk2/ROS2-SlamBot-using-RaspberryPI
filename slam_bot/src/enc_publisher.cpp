#include <unistd.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("encoder_publisher");
  auto pub = node->create_publisher<std_msgs::msg::Int16>("/enc_r_values", 10);
  std_msgs::msg::Int16 enc_r_msg;

  while (rclcpp::ok()) {
    enc_r_msg.data = 4;
    pub->publish(enc_r_msg);
    usleep(500000);
  }
  return 0;
}
