#pragma once

#include <atomic>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "wheelchair_sensor_msgs/msg/uwb.hpp"

class UWBSubscriber
{
public:
    UWBSubscriber(rclcpp::Node::SharedPtr node);



    float dist1() const noexcept { return dist1_.load(); }
    float dist2() const noexcept { return dist2_.load(); }
    float dist3() const noexcept { return dist3_.load(); }
    float dist4() const noexcept { return dist4_.load(); }
    float dist5() const noexcept { return dist5_.load(); }
    float dist6() const noexcept { return dist6_.load(); }

private:
   rclcpp::Node::SharedPtr node_;
   void msgCallback(const wheelchair_sensor_msgs::msg::UWB::SharedPtr msg);
   rclcpp::Subscription<wheelchair_sensor_msgs::msg::UWB>::SharedPtr sub_;
   
    std::atomic<float> dist1_{0.f}, dist2_{0.f}, dist3_{0.f},
                       dist4_{0.f}, dist5_{0.f}, dist6_{0.f};
};


