#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm> // for std::min and std::max
#include <cmath>
#include <string>

// Helper function to put values within a range.
double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

// Normalize an angle to the range [-180, 180] degrees.
double normalizeAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

// Replace this with the actual sensor reading.
double getDesiredHeading() {
    // For example, return 0.0 if we want to face north.
    return 0.0;
}

// Replace this with the actual sensor reading.
double getCurrentHeading() {
    // Example: current heading is measured as 10.0 degrees.
    return 10.0;
}

// Replace this with the logic to map overall speed to a base PWM percentage.
double getBasePWM() {
    // For instance, 3 mph might map to 100% PWM; here, we use 70%.
    return 70.0;
}

// Replace this with the actual motor control code.
void setMotorPWM(const std::string &motor, double pwm) {
    std::cout << "Motor " << motor << " PWM: " << pwm << "%" << std::endl;
}

int main() {
    // PID tuning parameters
    const double Kp = 1.0;
    const double Ki = 0.1;
    const double Kd = 0.05;

    // Anti-windup limits for the integral term
    const double max_integral = 100.0;
    const double min_integral = -100.0;

    // PID state variables
    double previous_error = 0.0;
    double integral = 0.0;

    // Control loop timing: 20 ms cycle (i.e., 50 Hz)
    const double dt = 0.02; // in seconds

    while (true) {
        // 1. Get the desired and current headings
        double desired_heading = getDesiredHeading();
        double current_heading = getCurrentHeading();

        // 2. Compute the error and normalize it
        double error = normalizeAngle(desired_heading - current_heading);

        // 3. Compute the proportional term
        double P = Kp * error;

        // 4. Compute the integral term with anti-windup clamping
        integral += error * dt;
        integral = clamp(integral, min_integral, max_integral);
        double I = Ki * integral;

        // 5. Compute the derivative term
        double derivative = (error - previous_error) / dt;
        double D = Kd * derivative;

        // Save the error for the next cycle
        previous_error = error;

        // 6. Total PID output is the turn offset
        double turn_offset = P + I + D;

        // 7. Get the base PWM for forward speed
        double base_pwm = getBasePWM();

        // 8. Adjust left and right motor PWM values based on the turn offset.
        // For a positive turn offset, left wheel gets more power and right less.
        double pwm_left  = clamp(base_pwm + turn_offset, 0.0, 100.0);
        double pwm_right = clamp(base_pwm - turn_offset, 0.0, 100.0);

        // 9. Output motor commands
        setMotorPWM("LEFT_MOTOR", pwm_left);
        setMotorPWM("RIGHT_MOTOR", pwm_right);

        // 10. Wait for the next cycle
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }

    return 0;
}
