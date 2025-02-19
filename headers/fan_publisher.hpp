//
// Created by Robbie on 2/12/25.
//

#ifndef FAN_PUBLISHER_HPP
#define FAN_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "wheelchair_sensor_msgs/msg/fan_speed.hpp"

struct FanSpeed
{
    int fan_percent_0;
    int fan_percent_1;
    int fan_percent_2;
    int fan_percent_3;
};

class FanPublisher
{
public:
    FanPublisher(rclcpp::Node::SharedPtr node);

    void trigger_publish(FanSpeed fanSpeed);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<wheelchair_sensor_msgs::msg::FanSpeed>::SharedPtr publisher_;
};

/**
class FanPublisher : public rclcpp::Node
{
public:
    FanPublisher();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<wheelchair_sensor_msgs::msg::FanSpeed>::SharedPtr publisher_;
    size_t count_;
};
**/

#endif // FAN_PUBLISHER_HPP
