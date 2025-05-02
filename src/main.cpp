#include "main.h"
#include "rclcpp/rclcpp.hpp"
#include "fan_publisher.hpp"
#include "light_publisher.hpp"
#include "sensors_subscriber.hpp"
#include "obstacle_subscriber.hpp"
#include "fingerprint_subscriber.hpp"
#include "ref_speed_publisher.hpp"

#include <atomic>
#include <thread>
#include <chrono>
#include <cstdint>

// ROS topic: true = clear, false = obstacle
std::atomic_bool front_clear{true};
std::atomic_bool back_clear{true};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wheelchair_code_module");

    auto sensors_subscriber      = std::make_shared<SensorsSubscriber>(node);
    auto obstacle_subscriber     = std::make_shared<ObstacleSubscriber>(node, front_clear, back_clear);
    auto fan_publisher           = std::make_shared<FanPublisher>(node);
    auto light_publisher         = std::make_shared<LightPublisher>(node);
    auto fingerprint_subscriber  = std::make_shared<FingerprintSubscriber>(node);
    auto ref_speed_publisher     = std::make_shared<RefSpeedPublisher>(node);

    std::thread spin_thread([&]() { rclcpp::spin(node); });
    RCLCPP_INFO(node->get_logger(), "Wheelchair node has started.");

    rclcpp::Rate rate(30);
    while (rclcpp::ok())
    {
        auto sd = sensors_subscriber->get_latest_sensor_data();

        RefSpeed ref_speed;
        //ref_speed.leftSpeed  = 25;
        //ref_speed.rightSpeed = 25;

        bool front_is_blocked = !front_clear.load(std::memory_order_relaxed);
        bool back_is_blocked  = !back_clear.load(std::memory_order_relaxed);

        RCLCPP_INFO(node->get_logger(),
            "[CMD] Joystick left=%d right=%d | front_clear=%d back_clear=%d | front_blocked=%d back_blocked=%d",
            ref_speed.leftSpeed, ref_speed.rightSpeed,
            front_clear.load(), back_clear.load(),
            front_is_blocked, back_is_blocked);

        if (front_is_blocked && back_is_blocked) {
            ref_speed.leftSpeed = 0;
            ref_speed.rightSpeed = 0;
            RCLCPP_DEBUG(node->get_logger(), "[SAFETY] Both blocked → STOP");
        }
        else if (front_is_blocked) {
            if (ref_speed.leftSpeed > 0)  ref_speed.leftSpeed = 0;
            if (ref_speed.rightSpeed > 0) ref_speed.rightSpeed = 0;
            RCLCPP_DEBUG(node->get_logger(), "[SAFETY] Front blocked → halt forward");
        }
        else if (back_is_blocked) {
            if (ref_speed.leftSpeed < 0)  ref_speed.leftSpeed = 0;
            if (ref_speed.rightSpeed < 0) ref_speed.rightSpeed = 0;
            RCLCPP_DEBUG(node->get_logger(), "[SAFETY] Back blocked → halt reverse");
        }

        ref_speed_publisher->trigger_publish(ref_speed);
        rate.sleep();
    }

    rclcpp::shutdown();
    spin_thread.join();
    RCLCPP_INFO(node->get_logger(), "Shutting down.");
    return 0;
}
