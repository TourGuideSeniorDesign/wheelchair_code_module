//
// Created by Angela on 5/18/25.
//

#include "uwb_subscriber.hpp"

UWBSubscriber::UWBSubscriber(rclcpp::Node::SharedPtr node)
: node_(node)
{
    sub_ = node_->create_subscription<wheelchair_sensor_msgs::msg::UWB>(
    "uwb_data", rclcpp::QoS(10), std::bind(&UWBSubscriber::msgCallback, this, std::placeholders::_1));
}


void UWBSubscriber::msgCallback(
    const wheelchair_sensor_msgs::msg::UWB::SharedPtr msg)
{
    dist1_.store(msg->dist1, std::memory_order_relaxed);
    dist2_.store(msg->dist2, std::memory_order_relaxed);
    dist3_.store(msg->dist3, std::memory_order_relaxed);
    dist4_.store(msg->dist4, std::memory_order_relaxed);
    dist5_.store(msg->dist5, std::memory_order_relaxed);
    dist6_.store(msg->dist6, std::memory_order_relaxed);

}
