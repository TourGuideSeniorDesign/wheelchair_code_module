//
// Created by leslier on 2/13/25.
//

#include "light_publisher.hpp"


LightPublisher::LightPublisher(rclcpp::Node::SharedPtr node)
: node_(node)
{
    publisher_ = node_->create_publisher<wheelchair_sensor_msgs::msg::Light>("light", 10);
}

void LightPublisher::trigger_publish(LightStates light_state) {
    auto message = wheelchair_sensor_msgs::msg::Light();
    message.state = static_cast<uint8_t>(light_state);
    this->publisher_->publish(message);
}