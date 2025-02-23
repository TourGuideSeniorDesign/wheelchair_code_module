//
// Created by Robbie on 2/14/25.
//

#ifndef FINGERPRINT_SUBSCRIBER_HPP
#define FINGERPRINT_SUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include "wheelchair_sensor_msgs/msg/fingerprint.hpp"

class FingerprintSubscriber
{
public:
    FingerprintSubscriber(rclcpp::Node::SharedPtr node);

private:
    void topic_callback(const wheelchair_sensor_msgs::msg::Fingerprint::SharedPtr msg) const;
    rclcpp::Subscription<wheelchair_sensor_msgs::msg::Fingerprint>::SharedPtr subscription_;
    rclcpp::Node::SharedPtr node_;
};

#endif //FINGERPRINT_SUBSCRIBER_HPP
