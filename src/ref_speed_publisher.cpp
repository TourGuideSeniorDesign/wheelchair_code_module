//
// Created by Robbie on 2/14/25.
//

#include "ref_speed_publisher.hpp"
#include <algorithm>

RefSpeedPublisher::RefSpeedPublisher(rclcpp::Node::SharedPtr node)
: node_(node)
{
    publisher_ = node_->create_publisher<wheelchair_sensor_msgs::msg::RefSpeed>("ref_speed", rclcpp::QoS(10).best_effort());
}

void RefSpeedPublisher::trigger_publish(const RefSpeed& ref_speed) {
    auto message = wheelchair_sensor_msgs::msg::RefSpeed();
    //int clamped_left = std::clamp(static_cast<int>(ref_speed.leftSpeed), -20, 20);
    //message.left_speed = static_cast<int8_t>(clamped_left);
    //int clamped_right = std::clamp(static_cast<int>(ref_speed.rightSpeed), -20, 20);
    //message.right_speed = static_cast<int8_t>(clamped_right);
    message.left_speed = ref_speed.leftSpeed;
    message.right_speed = ref_speed.rightSpeed;
    RCLCPP_INFO(node_->get_logger(), "Publishing Ref speed right: %u left: %u", message.right_speed, message.left_speed);
    this->publisher_->publish(message);
}
