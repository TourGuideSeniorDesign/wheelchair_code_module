#include "rclcpp/rclcpp.hpp"
#include "sensors_subscriber.h"  

#include <chrono>
#include <memory>
#include <string>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

// Speed constants 
const float SPEED_MAX     = 3.0f;  // Full speed
const float SPEED_CAUTION = 2.0f;  // Reduced speed
const float SPEED_STOP    = 0.0f;  // Stop

// IMU thresholds (example values)
const float ACC_LOW_THRESHOLD  = 7.0f;   // Below normal acceleration threshold
const float ACC_HIGH_THRESHOLD = 12.0f;  // Above normal acceleration threshold

// Finite State Machine (FSM) states
enum class RobotState {
  NORMAL,   // Full speed
  CAUTION,  // Reduced speed
  STOP      // Full stop
};

class StateMachine {
public:
  StateMachine() : current_state_(RobotState::NORMAL) {}

  // Update the FSM state based on the computed speed limit
  void update(float speed_limit) {
    if (speed_limit == SPEED_STOP) {
      current_state_ = RobotState::STOP;
    } else if (speed_limit == SPEED_CAUTION) {
      current_state_ = RobotState::CAUTION;
    } else {
      current_state_ = RobotState::NORMAL;
    }
  }

  std::string getStateName() const {
    switch (current_state_) {
      case RobotState::NORMAL:  return "NORMAL";
      case RobotState::CAUTION: return "CAUTION";
      case RobotState::STOP:    return "STOP";
      default:                  return "UNKNOWN";
    }
  }

private:
  RobotState current_state_;
};

class WheelchairNode : public rclcpp::Node {
public:
  WheelchairNode() : Node("wheelchair_node") {
    // Create a timer to periodically update the FSM using live sensor data (every 500ms)
    timer_ = this->create_wall_timer(500ms, std::bind(&WheelchairNode::updateFSM, this));
    RCLCPP_INFO(this->get_logger(), "Wheelchair node started.");
  }

  // Call this after the node is fully constructed
  void initialize() {
    // Create the sensor subscriber.
    // SensorsSubscriber is assumed to accept a rclcpp::Node::SharedPtr.
    sensor_subscriber_ = std::make_shared<SensorsSubscriber>(this->shared_from_this());
  }

private:
  // This method polls the latest sensor data, computes a decision, and updates the FSM.
  void updateFSM() {
    // Retrieve the latest sensor data.
    auto sensor_data = sensor_subscriber_->get_latest_sensor_data();

    // Ultrasonic Sensor Logic
    float ultrasonic_distance = sensor_data.ultrasonic_front_0;
    float ultrasonic_speed_limit = SPEED_MAX;
    if (ultrasonic_distance >= 1.0f) {
      ultrasonic_speed_limit = SPEED_MAX;
    } else if (ultrasonic_distance >= 0.5f) {
      ultrasonic_speed_limit = SPEED_CAUTION;
    } else {
      ultrasonic_speed_limit = SPEED_STOP;
    }

    // IMU Sensor Logic 
    // Compute acceleration magnitude from the IMU data.
    float ax = sensor_data.linear_acceleration_x;
    float ay = sensor_data.linear_acceleration_y;
    float az = sensor_data.linear_acceleration_z;
    float accMag = std::sqrt(ax * ax + ay * ay + az * az);
    float imu_speed_limit = SPEED_MAX;
    // If acceleration is too low or too high, assume a dangerous condition and require stop.
    if (accMag < ACC_LOW_THRESHOLD || accMag > ACC_HIGH_THRESHOLD) {
      imu_speed_limit = SPEED_STOP;
    }

    // Combine Decisions 
    float final_speed_limit = std::min(ultrasonic_speed_limit, imu_speed_limit);

    // Update the FSM with the final computed speed limit.
    fsm_.update(final_speed_limit);

    // Log the sensor readings, computed limits, and current FSM state.
    RCLCPP_INFO(this->get_logger(),
                "Ultrasonic: %.2f m, IMU acc: %.2f, Final Speed: %.2f, FSM State: %s",
                ultrasonic_distance, accMag, final_speed_limit, fsm_.getStateName().c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<SensorsSubscriber> sensor_subscriber_;
  StateMachine fsm_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  // Create the node using std::make_shared to allow shared_from_this()
  auto node = std::make_shared<WheelchairNode>();
  // Initialize the sensor subscriber now that the node is fully constructed.
  node->initialize();

  // Spin to process callbacks (both for the sensor subscriber and the timer)
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
