#include "speed_publisher.hpp"
#include <algorithm>
#include <cmath>

using std_msgs::msg::Bool;
using std_msgs::msg::Float32;

namespace wh = wheelchair;

wh::SpeedPublisher::SpeedPublisher(const std::string& out_topic,
                                   const rclcpp::QoS& qos)
: rclcpp::Node("speed_publisher")
{
  speed_pub_ = create_publisher<Float32>(out_topic, qos);
  
  obs_sub_ = create_subscription<Bool>(
      "obstacle_detected", 10,
      std::bind(&SpeedPublisher::obstacleCb, this, std::placeholders::_1));

  sensor_sub_ = std::make_shared<SensorsSubscriber>(this->shared_from_this());

  timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(DT * 1e3)),
      std::bind(&SpeedPublisher::timerCb, this));

  RCLCPP_INFO(get_logger(),
      "SpeedPublisher started — listening to obstacle_detected and sensors.");
}


void wh::SpeedPublisher::timerCb()
{

  auto s = sensor_sub_->get_latest_sensor_data();
  float ultrasonic = s.ultrasonic_front_0;
  float acc_mag    = std::hypot(s.linear_acceleration_x,
                     std::hypot(s.linear_acceleration_y,
                                s.linear_acceleration_z));
  bool pir_hit     = s.pir_front || s.pir_back ||
                     s.pir_left  || s.pir_right;


  float raw = fuseSpeeds(ultrasonic, acc_mag, pir_hit, obstacle_flag_);


  float max_delta = MAX_ACCEL * DT;
  float cmd = std::clamp(raw,
                         prev_speed_ - max_delta,
                         prev_speed_ + max_delta);
  prev_speed_ = cmd;


  Float32 out;
  out.data = cmd;
  speed_pub_->publish(out);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "[SPD] U=%.2f  A=%.2f  PIR=%s  OBS=%s  =>  %.2f m/s",
    ultrasonic, acc_mag,
    (pir_hit?"T":"F"), (obstacle_flag_?"T":"F"), cmd);
}


float wh::SpeedPublisher::fuseSpeeds(float ultrasonic,
                                     float acc_mag,
                                     bool  pir_hit,
                                     bool  obstacle_close) const
{
  float limit_ultr = (ultrasonic >= 1.0f) ? SPEED_MAX :
                     (ultrasonic >= 0.5f) ? SPEED_CAUTION
                                           : SPEED_STOP;

  float limit_imu  = (acc_mag < ACC_LOW_THR || acc_mag > ACC_HIGH_THR)
                   ? SPEED_STOP : SPEED_MAX;

  float limit_pir  = pir_hit ? SPEED_STOP : SPEED_MAX;

  float limit_lid  = obstacle_close ? SPEED_STOP : SPEED_MAX;

  return std::min({limit_ultr, limit_imu, limit_pir, limit_lid});
}



void wh::SpeedPublisher::obstacleCb(const Bool::SharedPtr msg)
{
  obstacle_flag_ = msg->data;
}
