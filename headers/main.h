#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Core>

struct BoundingBox {
    int cluster_id;
    Eigen::Vector3f min_pt;
    Eigen::Vector3f max_pt;
    float distance;   // Distance from LiDAR 
    float angle_deg;  // Angle (in degrees) around Z-axis (0=front, +left, -right)
};

// Speed constants (in mph) - example values
const float SPEED_MAX     = 3.0f;  // Full speed
const float SPEED_CAUTION = 2.0f;  // Caution
const float SPEED_SLOW    = 1.0f;  // Slow
const float SPEED_STOP    = 0.0f;  // Stop

// LiDAR thresholds (meters) for determining speed limit
const float LIDAR_FULL_THRESHOLD    = 3.0f;  
const float LIDAR_CAUTION_THRESHOLD = 1.0f;  

// Ultrasonic thresholds (meters)
const float ULTRASONIC_FULL_THRESHOLD    = 1.0f;
const float ULTRASONIC_CAUTION_THRESHOLD = 0.5f; 

// LiDAR sensor module
class LiDARSensor {
private:
    float obstacleDistance;  // Distance to nearest obstacle (m)
    float speedLimit;        // Computed speed limit based on LiDAR reading

public:
    LiDARSensor() 
      : obstacleDistance(100.0f), speedLimit(SPEED_MAX)
    {
    }

    // Manually update the obstacle distance, then compute speed limit
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
    bool  humanDetected; 
    float speedLimit;   

public:
    PIRSensor() 
      : humanDetected(false), speedLimit(SPEED_MAX)
    {
    }

    // If a human is detected, we command a full stop
    void updateDetection(bool detected) {
        humanDetected = detected;
        speedLimit    = (detected ? SPEED_STOP : SPEED_MAX);
    }
    
    float getSpeedLimit() const {
        return speedLimit;
    }
};

// Ultrasonic sensor module
class UltrasonicSensor {
private:
    float proximityDistance;  
    float speedLimit;         

public:
    UltrasonicSensor() 
      : proximityDistance(100.0f), speedLimit(SPEED_MAX)
    {
    }

    void updateDistance(float distance) {
        proximityDistance = distance;
        // Example thresholds for ultrasonic
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

// IMU sensor module
class IMUSensor {
private:
    float ax, ay, az;       // Current accelerations
    float prevAx, prevAy, prevAz;
    bool  danger;           
    
    // Configuration thresholds (example values)
    const float ACC_LOW_THRESHOLD   = 7.0f;   // Below normal gravity (~9.81)
    const float ACC_HIGH_THRESHOLD  = 12.0f;  // Above normal gravity
    const float FREEFALL_THRESHOLD  = 3.0f;   // near free-fall condition
    const float JERK_THRESHOLD      = 20.0f;  // example jerk threshold
    const float MIN_DT              = 0.01f;  // min time delta

    float fixedDeltaTime; // Example fixed time step

public:
    IMUSensor()
      : ax(0), ay(0), az(0),
        prevAx(0), prevAy(0), prevAz(0),
        danger(false),
        fixedDeltaTime(0.1f) // assume 10 Hz
    {
    }

    // Update with new raw acceleration values (m/s^2)
    void updateIMU(float newAx, float newAy, float newAz) {
        // Store previous acceleration
        prevAx = ax;
        prevAy = ay;
        prevAz = az;

        // Update current
        ax = newAx;
        ay = newAy;
        az = newAz;

        checkDanger();
    }

    // Determine if we are in a danger condition based on thresholds
    void checkDanger() {
        danger = false;

        // Overall Acceleration Magnitude
        float accMag = std::sqrt(ax*ax + ay*ay + az*az);
        if (accMag < ACC_LOW_THRESHOLD || accMag > ACC_HIGH_THRESHOLD) {
            danger = true;
            return;
        }

        // 2) Jerk = deltaAcceleration / deltaTime
        float dx = ax - prevAx;
        float dy = ay - prevAy;
        float dz = az - prevAz;
        float deltaAcc = std::sqrt(dx*dx + dy*dy + dz*dz);

        float dt = fixedDeltaTime;
        if (dt < MIN_DT) {
            dt = MIN_DT;
        }

        float jerk = deltaAcc / dt;
        if (jerk > JERK_THRESHOLD) {
            danger = true;
            return;
        }

        // 3) Free Fall Check
        if (accMag < FREEFALL_THRESHOLD) {
            danger = true;
            return;
        }

        // If none triggered, no danger
        danger = false;
    }

    float getSpeedLimit() const {
        return danger ? SPEED_STOP : SPEED_MAX;
    }
};

// SpeedController: fuses sensor outputs to compute final speed & steering
class SpeedController {
private:
    LiDARSensor      lidar;
    PIRSensor        pir;
    UltrasonicSensor ultrasonic;
    IMUSensor        imu;  

public:
    // Update each sensor with raw data (lidarDist, etc.)
    void updateSensors(float lidarDist, bool humanDetected, float ultrasonicDist,
                       float ax, float ay, float az)
    {
        lidar.updateDistance(lidarDist);
        pir.updateDetection(humanDetected);
        ultrasonic.updateDistance(ultrasonicDist);
        imu.updateIMU(ax, ay, az);
    }
    
    // Directly update LiDAR from bounding boxes: pick the closest bounding box
    void updateLidarFromBBoxes(const std::vector<BoundingBox>& bboxes) {
        float minDist = 100.0f; // some far default
        if (!bboxes.empty()) {
            for (auto & box : bboxes) {
                if (box.distance < minDist) {
                    minDist = box.distance;
                }
            }
        }
        // Now feed that minDist to the LiDARSensor
        lidar.updateDistance(minDist);
    }

    // After sensors are updated, compute final speed limit
    float computeFinalSpeed() {
        // The final speed is the most restrictive among all sensors
        return std::min({
            lidar.getSpeedLimit(),
            pir.getSpeedLimit(),
            ultrasonic.getSpeedLimit(),
            imu.getSpeedLimit()
        });
    }

    // Decide steering angle based on LiDAR bounding boxes
    // This example uses ±30° cone and 2m distance threshold
    float decideSteeringAngle(const std::vector<BoundingBox>& bboxes) {
        float frontConeDeg = 30.0f;
        float dangerDist   = 2.0f;
        float steerAngle   = 0.0f;  // Default: go straight

        for (auto &box : bboxes) {
            //
            if (box.distance < dangerDist && std::fabs(box.angle_deg) < frontConeDeg) {
                // If obstacle is in front cone
                if (box.angle_deg > 0.0f) {
                    // Obstacle left => steer right
                    steerAngle = -30.0f;
                } else {
                    // Obstacle right => steer left
                    steerAngle = 30.0f;
                }
                break; // Found a problem, set steering, done
            }
        }
        return steerAngle;
    }
};

#endif // MAIN_H
