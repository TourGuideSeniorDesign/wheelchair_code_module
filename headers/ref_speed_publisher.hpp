//
// Created by Robbie on 2/14/25.
//

#ifndef REF_SPEED_PUBLISHER_HPP
#define REF_SPEED_PUBLISHER_HPP

#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "wheelchair_sensor_msgs/msg/ref_speed.hpp"

struct RefSpeed{
  int8_t leftSpeed;
  int8_t rightSpeed;
 };

class RefSpeedPublisher {
public:
  RefSpeedPublisher(rclcpp::Node::SharedPtr node);

  void trigger_publish(RefSpeed ref_speed);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<wheelchair_sensor_msgs::msg::RefSpeed>::SharedPtr publisher_;
    };

#endif //REF_SPEED_PUBLISHER_HPP
