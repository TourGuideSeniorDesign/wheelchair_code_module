/* main.cpp – fixed to match ObstacleSubscriber(node, front, back, left, right) */

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

/* obstacle flags: true = clear, false = blocked */
std::atomic_bool front_clear{true};
std::atomic_bool back_clear{true};
std::atomic_bool left_turn_clear{true};
std::atomic_bool right_turn_clear{true};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wheelchair_code_module");

    auto sensors_subscriber  = std::make_shared<SensorsSubscriber>(node);
    auto obstacle_subscriber = std::make_shared<ObstacleSubscriber>(
            node,
            front_clear,
            back_clear,
            left_turn_clear,
            right_turn_clear);            // ← now passes all required flags
    auto fan_publisher          = std::make_shared<FanPublisher>(node);
    auto light_publisher        = std::make_shared<LightPublisher>(node);
    auto fingerprint_subscriber = std::make_shared<FingerprintSubscriber>(node);
    auto ref_speed_publisher    = std::make_shared<RefSpeedPublisher>(node);

    std::thread spin_thread([&]{ rclcpp::spin(node); });
    RCLCPP_INFO(node->get_logger(), "Wheelchair node has started.");

    rclcpp::Rate rate(30);
    while (rclcpp::ok())
    {
        /* base forward command */
        RefSpeed ref_speed{15, 15};

        bool front_blocked = !front_clear.load(std::memory_order_relaxed);
        bool back_blocked  = !back_clear.load(std::memory_order_relaxed);

        if (front_blocked && back_blocked) {                // both ways blocked
            ref_speed = {0, 0};
        } else if (front_blocked) {                         // stop forward
            if (ref_speed.leftSpeed  > 0) ref_speed.leftSpeed  = 0;
            if (ref_speed.rightSpeed > 0) ref_speed.rightSpeed = 0;
        } else if (back_blocked) {                          // stop reverse
            if (ref_speed.leftSpeed  < 0) ref_speed.leftSpeed  = 0;
            if (ref_speed.rightSpeed < 0) ref_speed.rightSpeed = 0;
        }

        ref_speed_publisher->trigger_publish(ref_speed);
        rate.sleep();
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}

