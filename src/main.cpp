#include "main.h"

int main() {
    SpeedController controller;

    // Scenario 1:
    // LiDAR detects an obstacle at 2.5 m,
    // No human detected via PIR,
    // Ultrasonic sensor reads 0.8 m (in caution range),
    controller.updateSensors(2.5f, false, 0.8f);
    float finalSpeed = controller.computeFinalSpeed();
    std::cout << "Scenario 1 - Final commanded speed: " << finalSpeed << " mph" << std::endl;

    // Scenario 2:
    // LiDAR detects an obstacle at 0.8 m,
    // A human is detected via PIR,
    // Ultrasonic sensor reads 0.4 m (immediate stop condition),
    controller.updateSensors(0.8f, true, 0.4f, 15.0f);
    finalSpeed = controller.computeFinalSpeed();
    std::cout << "Scenario 2 - Final commanded speed: " << finalSpeed << " mph" << std::endl;

    return 0;
}
