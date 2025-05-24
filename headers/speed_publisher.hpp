//
// Created by Angela on 4/18/25.
//


#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sensors_subscriber.h"          

namespace wheelchair
{

class SpeedPublisher : public rclcpp::Node
{
public:
  explicit SpeedPublisher(const std::string& out_topic = "speed_cmd",
                          const rclcpp::QoS& qos = rclcpp::QoS(10));

private:
  static constexpr float SPEED_MAX     = 3.0f;
  static constexpr float SPEED_CAUTION = 2.0f;
  static constexpr float SPEED_STOP    = 0.0f;
  static constexpr float ACC_LOW_THR   = 7.0f;
  static constexpr float ACC_HIGH_THR  = 12.0f;
  static constexpr float MAX_ACCEL     = 0.5f;  // m / s²
  static constexpr float DT            = 0.5f;  // timer period

  void timerCb();                                // main FSM timer
  void obstacleCb(const std_msgs::msg::Bool::SharedPtr);

  float fuseSpeeds(float ultrasonic,
                   float acc_mag,
                   bool  pir_hit,
                   bool  obstacle_close) const;

  std::shared_ptr<SensorsSubscriber> sensor_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obs_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool   obstacle_flag_{false};
  float  prev_speed_{0.0f};
};

} 
