// Created by Angela B. on 4/21/25 – split 4/28/25
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>

/* Listens to /front_clear and /back_clear and updates two atomic flags. */
class ObstacleSubscriber
{
public:
  ObstacleSubscriber(const rclcpp::Node::SharedPtr& node,
                     std::atomic_bool& front_clear_flag,
                     std::atomic_bool& back_clear_flag);

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_front_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_back_;
};
