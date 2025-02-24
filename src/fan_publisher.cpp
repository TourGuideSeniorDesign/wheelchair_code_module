//
// Created by aec117-fkmj9r3 on 2/12/25.
//

#include "fan_publisher.hpp"

using namespace std::chrono_literals;

FanPublisher::FanPublisher(rclcpp::Node::SharedPtr node)
: node_(node)
{
    publisher_ = node_->create_publisher<wheelchair_sensor_msgs::msg::FanSpeed>("fan_duty_cycles", 10);
}

void FanPublisher::trigger_publish(FanSpeed fanSpeed)
{
    auto message = wheelchair_sensor_msgs::msg::FanSpeed();
    message.fan_percent_0 = fanSpeed.fan_percent_0;
    message.fan_percent_1 = fanSpeed.fan_percent_1;
    message.fan_percent_2 = fanSpeed.fan_percent_2;
    message.fan_percent_3 = fanSpeed.fan_percent_3;
    RCLCPP_INFO(node_->get_logger(), "Publishing: fan_percent_0=%d, fan_percent_1=%d, fan_percent_2=%d, fan_percent_3=%d",
            message.fan_percent_0, message.fan_percent_1, message.fan_percent_2, message.fan_percent_3);
    publisher_->publish(message);
}

/**
FanPublisher::FanPublisher()
: Node("fan_publisher"), count_(0)
{
    publisher_ = this->create_publisher<wheelchair_sensor_msgs::msg::FanSpeed>("fan_duty_cycles", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&FanPublisher::timer_callback, this));
}

void FanPublisher::timer_callback()
{
    auto message = wheelchair_sensor_msgs::msg::FanSpeed();
    message.fan_percent_0 = 0;
    message.fan_percent_1 = 0;
    message.fan_percent_2 = 0;
    message.fan_percent_3 = 0;
    RCLCPP_INFO(this->get_logger(), "Publishing: fan_percent_0=%d, fan_percent_1=%d, fan_percent_2=%d, fan_percent_3=%d",
            message.fan_percent_0, message.fan_percent_1, message.fan_percent_2, message.fan_percent_3);
    publisher_->publish(message);
}
**/
