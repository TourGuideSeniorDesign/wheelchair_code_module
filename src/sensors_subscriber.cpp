//
// Created by Robbie on 2/12/25.
//

#include "sensors_subscriber.hpp"

SensorsSubscriber::SensorsSubscriber(rclcpp::Node::SharedPtr node)
: node_(node)
{
  subscription_ = node_->create_subscription<wheelchair_sensor_msgs::msg::Sensors>(
    "sensors", rclcpp::QoS(10).best_effort(), std::bind(&SensorsSubscriber::topic_callback, this, std::placeholders::_1));
}

void SensorsSubscriber::topic_callback(const wheelchair_sensor_msgs::msg::Sensors::SharedPtr msg) const
{
    latest_sensor_data_.left_speed = msg->left_speed;
    latest_sensor_data_.right_speed = msg->right_speed;
    latest_sensor_data_.ultrasonic_front_0 = msg->ultrasonic_front_0;
    latest_sensor_data_.ultrasonic_front_1 = msg->ultrasonic_front_1;
    latest_sensor_data_.ultrasonic_back = msg->ultrasonic_back;
    latest_sensor_data_.ultrasonic_left = msg->ultrasonic_left;
    latest_sensor_data_.ultrasonic_right = msg->ultrasonic_right;
    latest_sensor_data_.pir_front = msg->pir_front;
    latest_sensor_data_.pir_back = msg->pir_back;
    latest_sensor_data_.pir_left = msg->pir_left;
    latest_sensor_data_.pir_right = msg->pir_right;
    latest_sensor_data_.fan_speed_0 = msg->fan_speed_0;
    latest_sensor_data_.fan_speed_1 = msg->fan_speed_1;
    latest_sensor_data_.fan_speed_2 = msg->fan_speed_2;
    latest_sensor_data_.fan_speed_3 = msg->fan_speed_3;
    latest_sensor_data_.linear_acceleration_x = msg->linear_acceleration_x;
    latest_sensor_data_.linear_acceleration_y = msg->linear_acceleration_y;
    latest_sensor_data_.linear_acceleration_z = msg->linear_acceleration_z;
    latest_sensor_data_.angular_velocity_x = msg->angular_velocity_x;
    latest_sensor_data_.angular_velocity_y = msg->angular_velocity_y;
    latest_sensor_data_.angular_velocity_z = msg->angular_velocity_z;
    latest_sensor_data_.magnetic_field_x = msg->magnetic_field_x;
    latest_sensor_data_.magnetic_field_y = msg->magnetic_field_y;
    latest_sensor_data_.magnetic_field_z = msg->magnetic_field_z;

//  RCLCPP_INFO(node_->get_logger(),
//  "I heard:\n"
//  "left_speed=%d\n"
//  "right_speed=%d\n"
//  "ultrasonic_front_0=%d\n"
//  "ultrasonic_front_1=%d\n"
//  "ultrasonic_back=%d\n"
//  "ultrasonic_left=%d\n"
//  "ultrasonic_right=%d\n"
//  "pir_front=%s\n"
//  "pir_back=%s\n"
//  "pir_left=%s\n"
//  "pir_right=%s\n"
//  "fan_speed_0=%d\n"
//  "fan_speed_1=%d\n"
//  "fan_speed_2=%d\n"
//  "fan_speed_3=%d\n"
//  "linear_acceleration_x=%.2f\n"
//  "linear_acceleration_y=%.2f\n"
//  "linear_acceleration_z=%.2f\n"
//  "angular_velocity_x=%.2f\n"
//  "angular_velocity_y=%.2f\n"
//  "angular_velocity_z=%.2f\n"
//  "magnetic_field_x=%.2f\n"
//  "magnetic_field_y=%.2f\n"
//  "magnetic_field_z=%.2f",
//  msg->left_speed, msg->right_speed, msg->ultrasonic_front_0, msg->ultrasonic_front_1, msg->ultrasonic_back, msg->ultrasonic_left, msg->ultrasonic_right,
//  msg->pir_front ? "true" : "false", msg->pir_back ? "true" : "false", msg->pir_left ? "true" : "false", msg->pir_right ? "true" : "false",
//  msg->fan_speed_0, msg->fan_speed_1, msg->fan_speed_2, msg->fan_speed_3,
//  msg->linear_acceleration_x, msg->linear_acceleration_y, msg->linear_acceleration_z, msg->angular_velocity_x, msg->angular_velocity_y, msg->angular_velocity_z,
//  msg->magnetic_field_x, msg->magnetic_field_y, msg->magnetic_field_z);
  }

SensorData SensorsSubscriber::get_latest_sensor_data() const
{
    return latest_sensor_data_;
}