#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <algorithm>
#include <cmath>

// Speed constants (in mph)
const float SPEED_MAX    = 3.0f;  // Full speed
const float SPEED_CAUTION = 2.0f; // Caution
const float SPEED_SLOW   = 1.0f;  // Slow
const float SPEED_STOP   = 0.0f;  // Stop

// LiDAR thresholds (in meters)
const float LIDAR_FULL_THRESHOLD    = 3.0f;  
const float LIDAR_CAUTION_THRESHOLD = 1.0f;  

// Ultrasonic thresholds (in meters)
const float ULTRASONIC_FULL_THRESHOLD    = 1.0f;
const float ULTRASONIC_CAUTION_THRESHOLD = 0.5f; 

// --------------------------------------------------
// LiDAR sensor module
// --------------------------------------------------
class LiDARSensor {
private:
    float obstacleDistance;  // Distance to nearest obstacle (m)
    float speedLimit;        // Computed speed limit based on LiDAR reading

public:
    LiDARSensor() : obstacleDistance(100.0f), speedLimit(SPEED_MAX) {}

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

// --------------------------------------------------
// PIR sensor module
// --------------------------------------------------
class PIRSensor {
private:
    bool  humanDetected; 
    float speedLimit;   

public:
    PIRSensor() : humanDetected(false), speedLimit(SPEED_MAX) {}

    // If a human is detected, we command a full stop.
    void updateDetection(bool detected) {
        humanDetected = detected;
        speedLimit = humanDetected ? SPEED_STOP : SPEED_MAX;
    }
    
    float getSpeedLimit() const {
        return speedLimit;
    }
};

// --------------------------------------------------
// Ultrasonic sensor module
// --------------------------------------------------
class UltrasonicSensor {
private:
    float proximityDistance;  
    float speedLimit;         

public:
    UltrasonicSensor() : proximityDistance(100.0f), speedLimit(SPEED_MAX) {}

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

// --------------------------------------------------
// IMU sensor module
// --------------------------------------------------
class IMUSensor {
private:
    float ax, ay, az;       // Current accelerations
    float prevAx, prevAy, prevAz;
    bool  danger;           // True if we detect a danger condition
    
    // Configuration thresholds (tune as needed)
    const float ACC_LOW_THRESHOLD   = 7.0f;   // Below normal gravity
    const float ACC_HIGH_THRESHOLD  = 12.0f;  // Above normal gravity
    const float FREEFALL_THRESHOLD  = 3.0f;   // ~2-3 m/s^2 indicates near free-fall
    const float JERK_THRESHOLD      = 20.0f;  // Example jerk threshold
    const float MIN_DT              = 0.01f;  // Minimum time delta to avoid divide-by-zero

    // For a simple approach without a “time” parameter,
    // we’ll just assume we get a new reading at a fixed interval (e.g. dt=0.1s).
    // In a real system, you’d pass an actual timestamp or dt to updateIMU.
    float fixedDeltaTime;

public:
    IMUSensor()
        : ax(0), ay(0), az(0),
          prevAx(0), prevAy(0), prevAz(0),
          danger(false),
          fixedDeltaTime(0.1f) // assume 10 Hz reading
    {}

    // Update with new raw acceleration values (m/s^2)
    void updateIMU(float newAx, float newAy, float newAz) {
        // Store previous acceleration
        prevAx = ax;
        prevAy = ay;
        prevAz = az;

        // Update current acceleration
        ax = newAx;
        ay = newAy;
        az = newAz;

        checkDanger();
    }

    // Determine if we are in a danger condition based on the logic
    void checkDanger() {
        danger = false;

        // 1. Overall Acceleration Magnitude
        float accMag = std::sqrt(ax*ax + ay*ay + az*az);
        if (accMag < ACC_LOW_THRESHOLD || accMag > ACC_HIGH_THRESHOLD) {
            // Danger if outside normal gravity range
            danger = true;
            return;
        }

        // 2. Change of Acceleration Over Time (Jerk)
        //    jerk = deltaAcceleration / deltaTime
        float dx = ax - prevAx;
        float dy = ay - prevAy;
        float dz = az - prevAz;
        float deltaAcc = std::sqrt(dx*dx + dy*dy + dz*dz);

        float dt = fixedDeltaTime; // or actual time difference if available
        if (dt < MIN_DT) {
            dt = MIN_DT; // avoid division by zero
        }

        float jerk = deltaAcc / dt;
        if (jerk > JERK_THRESHOLD) {
            danger = true;
            return;
        }

        // 3. Free Fall Check
        if (accMag < FREEFALL_THRESHOLD) {
            danger = true;
            return;
        }

        danger = false;
    }

    float getSpeedLimit() const {
        return (danger) ? SPEED_STOP : SPEED_MAX;
    }
};

// --------------------------------------------------
// SpeedController fuses sensor outputs to compute final speed
// --------------------------------------------------
class SpeedController {
private:
    LiDARSensor      lidar;
    PIRSensor        pir;
    UltrasonicSensor ultrasonic;
    IMUSensor        imu;  

public:
    void updateSensors(float lidarDist, bool humanDetected, float ultrasonicDist,
                       float ax, float ay, float az)
    {
        lidar.updateDistance(lidarDist);
        pir.updateDetection(humanDetected);
        ultrasonic.updateDistance(ultrasonicDist);
        imu.updateIMU(ax, ay, az);
    }
    
    // Compute final commanded speed: minimum of all sensor speed limits
    float computeFinalSpeed() {
        return std::min({ lidar.getSpeedLimit(),
                          pir.getSpeedLimit(),
                          ultrasonic.getSpeedLimit(),
                          imu.getSpeedLimit() }); // Include IMU limit
    }
};

#endif // MAIN_H
