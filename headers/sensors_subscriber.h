//
// Created by Robbie on 2/12/25.
//

#ifndef SENSOR_SUBSCRIBER_HPP
#define SENSOR_SUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include "wheelchair_sensor_msgs/msg/sensors.hpp"

struct SensorData {
    int left_speed;
    int right_speed;
    int ultrasonic_front_0;
    int ultrasonic_front_1;
    int ultrasonic_back;
    int ultrasonic_left;
    int ultrasonic_right;
    bool pir_front;
    bool pir_back;
    bool pir_left;
    bool pir_right;
    int fan_speed_0;
    int fan_speed_1;
    int fan_speed_2;
    int fan_speed_3;
    float linear_acceleration_x;
    float linear_acceleration_y;
    float linear_acceleration_z;
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    float magnetic_field_x;
    float magnetic_field_y;
    float magnetic_field_z;
};

class SensorsSubscriber
{
public:
    SensorsSubscriber(rclcpp::Node::SharedPtr node);
    SensorData get_latest_sensor_data() const;

private:
    void topic_callback(const wheelchair_sensor_msgs::msg::Sensors::SharedPtr msg) const;
    rclcpp::Subscription<wheelchair_sensor_msgs::msg::Sensors>::SharedPtr subscription_;
    rclcpp::Node::SharedPtr node_;
    mutable SensorData latest_sensor_data_;
};

#endif // SENSOR_SUBSCRIBER_HPP
