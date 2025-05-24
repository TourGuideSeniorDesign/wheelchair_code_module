// main.cpp  – 2025-05-08 pivot-enhanced
#include "main.h"
#include "rclcpp/rclcpp.hpp"
#include "fan_publisher.hpp"
#include "light_publisher.hpp"
#include "sensors_subscriber.hpp"
#include "obstacle_subscriber.hpp"
#include "fingerprint_subscriber.hpp"
#include "ref_speed_publisher.hpp"

#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstdint>

constexpr int PIVOT_SPEED = 25;         


std::atomic_bool front_clear{true};
std::atomic_bool back_clear{true};
std::atomic_bool left_turn_clear{true};   
std::atomic_bool right_turn_clear{true};  

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

    /* cone-clear subscriptions (fire-and-forget lambdas) */
    auto left_sub  = node->create_subscription<std_msgs::msg::Bool>(
        "left_turn_clear", 10,
        [](std_msgs::msg::Bool::SharedPtr m){
            left_turn_clear.store(m->data, std::memory_order_relaxed);
        });
    auto right_sub = node->create_subscription<std_msgs::msg::Bool>(
        "right_turn_clear", 10,
        [](std_msgs::msg::Bool::SharedPtr m){
            right_turn_clear.store(m->data, std::memory_order_relaxed);
        });

    std::thread spin_thread([&]() { rclcpp::spin(node); });
    RCLCPP_INFO(node->get_logger(), "Wheelchair node has started.");

    rclcpp::Rate rate(30);
    while (rclcpp::ok())
    {
        auto sd = sensors_subscriber->get_latest_sensor_data();

        /* joystick requests */
        RefSpeed ref_speed;
        ref_speed.leftSpeed  = sd.left_speed;
        ref_speed.rightSpeed = sd.right_speed;

        bool front_is_blocked = !front_clear.load(std::memory_order_relaxed);
        bool back_is_blocked  = !back_clear .load(std::memory_order_relaxed);

        if (front_is_blocked && back_is_blocked) {
            ref_speed.leftSpeed = ref_speed.rightSpeed = 0;
        }
        else if (front_is_blocked) {
            /* pivot around obstacle */
            bool right_ok = right_turn_clear.load(std::memory_order_relaxed);
            bool left_ok  = left_turn_clear .load(std::memory_order_relaxed);

            if (right_ok) {                  // clockwise turn
                ref_speed.leftSpeed  = 0;
                ref_speed.rightSpeed =  PIVOT_SPEED;
            }
            else if (left_ok) {              // counter-clockwise turn
                ref_speed.leftSpeed  =  PIVOT_SPEED;
                ref_speed.rightSpeed = 0;
            }
            else {                           // cones both blocked → stop forward
                if (ref_speed.leftSpeed > 0)  ref_speed.leftSpeed  = 0; 
                if (ref_speed.rightSpeed > 0) ref_speed.rightSpeed = 0;
            }
        }
        else if (back_is_blocked) {
            if (ref_speed.leftSpeed  < 0) ref_speed.leftSpeed  = 0;
            if (ref_speed.rightSpeed < 0) ref_speed.rightSpeed = 0;
        }

        RCLCPP_INFO(node->get_logger(),
            "[CMD] Joy L=%d R=%d | front=%d back=%d | coneL=%d coneR=%d",
            ref_speed.leftSpeed, ref_speed.rightSpeed,
            front_clear.load(), back_clear.load(),
            left_turn_clear.load(), right_turn_clear.load());

        ref_speed_publisher->trigger_publish(ref_speed);
        rate.sleep();
    }

    rclcpp::shutdown();
    spin_thread.join();
    RCLCPP_INFO(node->get_logger(), "Shutting down.");
    return 0;
}
