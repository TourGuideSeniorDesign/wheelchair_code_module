#include "main.h"
#include "rclcpp/rclcpp.hpp"
#include "fan_publisher.hpp"
#include "light_publisher.hpp"
#include "sensors_subscriber.hpp"
#include "fingerprint_subscriber.hpp"
#include "ref_speed_publisher.hpp"
#include <thread>

int main(int argc, char *argv[]) {
    // Initialize the ROS 2 node and setting up subscribers and publishers
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wheelchair_code_module");
    auto fan_publisher = std::make_shared<FanPublisher>(node);
    auto light_publisher = std::make_shared<LightPublisher>(node);
    auto sensors_subscriber = std::make_shared<SensorsSubscriber>(node);
    auto fingerprint_subscriber = std::make_shared<FingerprintSubscriber>(node);
    auto ref_speed_publisher = std::make_shared<RefSpeedPublisher>(node);

    std::thread spin_thread([&]() {
    rclcpp::spin(node);
    });

    RCLCPP_INFO(node->get_logger(), "Wheelchair node has started.");

    SpeedController controller;

    //Loop used to get data while the node is running
    while (rclcpp::ok()){
      // Example to show how to get the latest sensor data
      //Right now it just grabs the sensor data and publishes the joystick speeds on the ref_speed topic
        auto sensor_data = sensors_subscriber->get_latest_sensor_data();
        RefSpeed ref_speed;
        ref_speed.leftSpeed = sensor_data.left_speed;
        ref_speed.rightSpeed = sensor_data.right_speed;
        ref_speed_publisher->trigger_publish(ref_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
     }

    // Scenario 1:
    // LiDAR detects an obstacle at 2.5 m,
    // No human detected via PIR,
    // Ultrasonic sensor reads 0.8 m (in caution range),
//    controller.updateSensors(2.5f, false, 0.8f);
//    float finalSpeed = controller.computeFinalSpeed();
//    std::cout << "Scenario 1 - Final commanded speed: " << finalSpeed << " mph" << std::endl;

    // Scenario 2:
    // LiDAR detects an obstacle at 0.8 m,
    // A human is detected via PIR,
    // Ultrasonic sensor reads 0.4 m (immediate stop condition),
//    controller.updateSensors(0.8f, true, 0.4f, 15.0f);
//    finalSpeed = controller.computeFinalSpeed();
//    std::cout << "Scenario 2 - Final commanded speed: " << finalSpeed << " mph" << std::endl;


    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
