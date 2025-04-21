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

    std::thread spin_thread([&]() {
    rclcpp::spin(node);
    });

    // Set the rate to 10 Hz
    rclcpp::Rate rate(10);

    RCLCPP_INFO(node->get_logger(), "Monitoring CPU temperature (press Ctrl+C to stop):");
    while (rclcpp::ok()) {
        double tempC = readTemperature(tempPath);
        if (tempC < 0) {
            RCLCPP_INFO(node->get_logger(), "Failed to read temperature!");
        } else {
            RCLCPP_INFO(node->get_logger(), "CPU Temperature: %f °C", tempC);

            if (tempC < 30.0) {
                // Handle case for temperatures below 30°C
                FanSpeed fanSpeed;
                fanSpeed.fan_percent_0 = 0;
                fanSpeed.fan_percent_1 = 0;
                fanSpeed.fan_percent_2 = 0;
                fanSpeed.fan_percent_3 = 0;
                fan_publisher->trigger_publish(fanSpeed);
            } else if (tempC >= 30.0 && tempC < 40.0) {
                // Handle case for temperatures between 30°C and 50°C
                FanSpeed fanSpeed;
                fanSpeed.fan_percent_0 = 25;
                fanSpeed.fan_percent_1 = 25;
                fanSpeed.fan_percent_2 = 25;
                fanSpeed.fan_percent_3 = 25;
                fan_publisher->trigger_publish(fanSpeed);
            } else if (tempC >= 40.0 && tempC < 50) {
                // Handle case for temperatures above 50°C
                FanSpeed fanSpeed;
                fanSpeed.fan_percent_0 = 50;
                fanSpeed.fan_percent_1 = 50;
                fanSpeed.fan_percent_2 = 50;
                fanSpeed.fan_percent_3 = 50;
                fan_publisher->trigger_publish(fanSpeed);
            } else if (tempC >= 50.0){
                FanSpeed fanSpeed;
                fanSpeed.fan_percent_0 = 75;
                fanSpeed.fan_percent_1 = 75;
                fanSpeed.fan_percent_2 = 75;
                fanSpeed.fan_percent_3 = 75;
                fan_publisher->trigger_publish(fanSpeed);
            }
        }

        // sleep to maintain the rate
        rate.sleep();
    }

    return 0;
}
