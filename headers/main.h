#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <algorithm>
#include <cmath>

// Speed constants (in mph)
const float SPEED_MAX    = 3.0f;  // Full speed: 3 mph
const float SPEED_CAUTION = 2.0f; // Caution: 2 mph
const float SPEED_SLOW   = 1.0f;  // Slow: 1 mph
const float SPEED_STOP   = 0.0f;  // Stop: 0 mph

// LiDAR thresholds (in meters)
const float LIDAR_FULL_THRESHOLD    = 3.0f;  // Obstacle at or beyond 3 m: no restriction (full speed)
const float LIDAR_CAUTION_THRESHOLD = 1.0f;  // Obstacle between 1 and 3 m: caution speed; below 1 m: slow

// Ultrasonic thresholds (in meters)
const float ULTRASONIC_FULL_THRESHOLD    = 1.0f;  // Beyond 1 m: full speed
const float ULTRASONIC_CAUTION_THRESHOLD = 0.5f;  // Between 0.5 m and 1 m: caution; below 0.5 m: stop

// LiDAR sensor module
class LiDARSensor {
private:
    float obstacleDistance;  // Distance to nearest obstacle (meters)
    float speedLimit;        // Computed speed limit based on LiDAR reading
public:
    LiDARSensor() : obstacleDistance(100.0f), speedLimit(SPEED_MAX) {}

    // Update the LiDAR distance reading
    void updateDistance(float distance) {
        obstacleDistance = distance;
        if (obstacleDistance >= LIDAR_FULL_THRESHOLD) {
            speedLimit = SPEED_MAX;
        } else if (obstacleDistance >= LIDAR_CAUTION_THRESHOLD) {
            speedLimit = SPEED_CAUTION;
        } else {
            speedLimit = SPEED_SLOW;
        }
    }
    
    float getSpeedLimit() const {
        return speedLimit;
    }
};

// PIR sensor module
class PIRSensor {
private:
    bool humanDetected; // True if a human is detected
    float speedLimit;   // Computed speed limit based on PIR sensor
public:
    PIRSensor() : humanDetected(false), speedLimit(SPEED_MAX) {}

    // Update the PIR detection status.
    // (For simplicity, if a human is detected, we command a full stop.)
    void updateDetection(bool detected) {
        humanDetected = detected;
        speedLimit = humanDetected ? SPEED_STOP : SPEED_MAX;
    }
    
    float getSpeedLimit() const {
        return speedLimit;
    }
};

// Ultrasonic sensor module
class UltrasonicSensor {
private:
    float proximityDistance;  // Measured distance from ultrasonic sensor (meters)
    float speedLimit;         // Computed speed limit based on ultrasonic reading
public:
    UltrasonicSensor() : proximityDistance(100.0f), speedLimit(SPEED_MAX) {}

    // Update the ultrasonic sensor reading
    void updateDistance(float distance) {
        proximityDistance = distance;
        if (proximityDistance >= ULTRASONIC_FULL_THRESHOLD) {
            speedLimit = SPEED_MAX;
        } else if (proximityDistance >= ULTRASONIC_CAUTION_THRESHOLD) {
            speedLimit = SPEED_CAUTION;
        } else {
            speedLimit = SPEED_STOP;
        }
    }
    
    float getSpeedLimit() const {
        return speedLimit;
    }
};


// SpeedController fuses sensor outputs to compute final speed
class SpeedController {
private:
    LiDARSensor       lidar;
    PIRSensor         pir;
    UltrasonicSensor  ultrasonic;
    IMUSensor         imu;
public:
    // Update all sensor readings
    void updateSensors(float lidarDist, bool humanDetected, float ultrasonicDist, float imuAngle) {
        lidar.updateDistance(lidarDist);
        pir.updateDetection(humanDetected);
        ultrasonic.updateDistance(ultrasonicDist);
    }
    
    // Compute final commanded speed: minimum of all sensor speed limits
    float computeFinalSpeed() {
        return std::min({ lidar.getSpeedLimit(),
                           pir.getSpeedLimit(),
                           ultrasonic.getSpeedLimit()});
    }
};

#endif // MAIN_H
