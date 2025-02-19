//
// Created by Robbie on 2/14/25.
//

#include "fingerprint_subscriber.hpp"

FingerprintSubscriber::FingerprintSubscriber(rclcpp::Node::SharedPtr node)
: node_(node)
{
    subscription_ = node_->create_subscription<wheelchair_sensor_msgs::msg::Fingerprint>(
      "fingerprint", rclcpp::QoS(10).best_effort(), std::bind(&FingerprintSubscriber::topic_callback, this, std::placeholders::_1));
}

void FingerprintSubscriber::topic_callback(const wheelchair_sensor_msgs::msg::Fingerprint::SharedPtr msg) const
{
    RCLCPP_INFO(node_->get_logger(), "I heard: fingerprint=%d", msg->user_id);
}