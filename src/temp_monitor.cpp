//
// Created by Robbie on 4/21/25.
//

#include "temp_monitor.hpp"
#include "fan_publisher.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

std::atomic_bool running(true);

void handle_sigint(int)
{
    running = false;  // causes main loop to exit
}

namespace fs = std::filesystem;

std::string findCpuThermalZone() {
    const std::string basePath = "/sys/class/thermal/";

    for (const auto& entry : fs::directory_iterator(basePath)) {
        if (entry.is_directory() && entry.path().filename().string().find("thermal_zone") == 0) {
            std::ifstream typeFile(entry.path() / "type");
            std::string type;
            if (typeFile.is_open()) {
                std::getline(typeFile, type);
                typeFile.close();

                if (type.find("CPU") != std::string::npos || type.find("cpu") != std::string::npos) {
                    return (entry.path() / "temp").string();
                }
            }
        }
    }
    return "";
}

double readTemperature(const std::string& tempPath) {
    std::ifstream tempFile(tempPath);
    int tempMilliC = 0;
    if (tempFile.is_open()) {
        tempFile >> tempMilliC;
        tempFile.close();
        return tempMilliC / 1000.0;
    }
    return -1.0;
}

int main(int argc, char** argv) {
    std::string tempPath = findCpuThermalZone();
    if (tempPath.empty()) {
        std::cerr << "CPU thermal zone not found." << std::endl;
        return 1;
    }

    // Initialize the ROS 2 node and setting up subscribers and publishers
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("temperature_monitor");
    auto fan_publisher = std::make_shared<FanPublisher>(node);

    std::signal(SIGINT, handle_sigint);

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);

    // Thread that runs the executor manually
    std::thread spin_thread([&]() {
        while (rclcpp::ok() && running) {
            executor->spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    // Set the rate to 10 Hz
    rclcpp::Rate rate(10);

    RCLCPP_INFO(node->get_logger(), "Monitoring CPU temperature (press Ctrl+C to stop):");
    while (rclcpp::ok() && running) {
        double tempC = readTemperature(tempPath);
        if (tempC < 0) {
            RCLCPP_INFO(node->get_logger(), "Failed to read temperature!");
        } else {
            RCLCPP_INFO(node->get_logger(), "CPU Temperature: %f °C", tempC);

            FanSpeed newFanSpeed;

            if (tempC < 30.0) {
                newFanSpeed.fan_percent_0 = 0;
                newFanSpeed.fan_percent_1 = 0;
                newFanSpeed.fan_percent_2 = 0;
                newFanSpeed.fan_percent_3 = 0;
            } else if (tempC < 40.0) {
                newFanSpeed.fan_percent_0 = 25;
                newFanSpeed.fan_percent_1 = 25;
                newFanSpeed.fan_percent_2 = 25;
                newFanSpeed.fan_percent_3 = 25;
            } else if (tempC < 50.0) {
                newFanSpeed.fan_percent_0 = 50;
                newFanSpeed.fan_percent_1 = 50;
                newFanSpeed.fan_percent_2 = 50;
                newFanSpeed.fan_percent_3 = 50;
            } else {
                newFanSpeed.fan_percent_0 = 75;
                newFanSpeed.fan_percent_1 = 75;
                newFanSpeed.fan_percent_2 = 75;
                newFanSpeed.fan_percent_3 = 75;
            }

            // Static flag to ensure first publish happens
            static bool first_run = true;
            static FanSpeed lastFanSpeed;

            if (first_run ||
                newFanSpeed.fan_percent_0 != lastFanSpeed.fan_percent_0 ||
                newFanSpeed.fan_percent_1 != lastFanSpeed.fan_percent_1 ||
                newFanSpeed.fan_percent_2 != lastFanSpeed.fan_percent_2 ||
                newFanSpeed.fan_percent_3 != lastFanSpeed.fan_percent_3) {

                fan_publisher->trigger_publish(newFanSpeed);
                lastFanSpeed = newFanSpeed;
                first_run = false;
            }
        }

        // sleep to maintain the rate
        rate.sleep();
    }

    FanSpeed off;
    off.fan_percent_0 = 0;
    off.fan_percent_1 = 0;
    off.fan_percent_2 = 0;
    off.fan_percent_3 = 0;
    fan_publisher->trigger_publish(off);
    RCLCPP_INFO(rclcpp::get_logger("shutdown"), "Shutdown: Turning off fans.");

    // Give time to publish and flush
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Cleanup
    executor->cancel();  // This ensures executor thread stops
    rclcpp::shutdown();
    spin_thread.join();


    return 0;
}
