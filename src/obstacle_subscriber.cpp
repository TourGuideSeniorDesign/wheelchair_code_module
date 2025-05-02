#include "obstacle_subscriber.hpp"

ObstacleSubscriber::ObstacleSubscriber(const rclcpp::Node::SharedPtr& node,
                                       std::atomic_bool& front_clear_flag,
                                       std::atomic_bool& back_clear_flag)
{
  sub_front_ = node->create_subscription<std_msgs::msg::Bool>(
      "front_clear", 10,
      [&](const std_msgs::msg::Bool::SharedPtr m)
      {
        front_clear_flag.store(m->data, std::memory_order_relaxed);
      });

  sub_back_ = node->create_subscription<std_msgs::msg::Bool>(
      "back_clear", 10,
      [&](const std_msgs::msg::Bool::SharedPtr m)
      {
        back_clear_flag.store(m->data, std::memory_order_relaxed);
      });

  RCLCPP_INFO(node->get_logger(),
              "ObstacleSubscriber: front_clear & back_clear wired up.");
}
