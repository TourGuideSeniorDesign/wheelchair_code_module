// speed_publisher.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class SpeedPublisher : public rclcpp::Node
{
public:
  SpeedPublisher()
  : Node("speed_publisher")
  {
    speed_pub_ = create_publisher<std_msgs::msg::Float32>("speed_cmd", 10);
  }

  void publish(float v)
  {
    std_msgs::msg::Float32 msg;
    msg.data = v;
    speed_pub_->publish(msg);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
};

