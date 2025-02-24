//
// Created by leslier on 2/13/25.
//

#ifndef LIGHTPUBLISHER_H
#define LIGHTPUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "wheelchair_sensor_msgs/msg/light.hpp"

enum class LightStates{
  OFF = 0,
  STEADY = 1,
  FLASHING = 2,
  };

class LightPublisher {
public:
  LightPublisher(rclcpp::Node::SharedPtr node);

  void trigger_publish(LightStates light_state);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<wheelchair_sensor_msgs::msg::Light>::SharedPtr publisher_;
};

#endif //LIGHTPUBLISHER_H
