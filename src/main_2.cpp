#include "rclcpp/rclcpp.hpp"
#include "sensors_subscriber.h"  

#include <chrono>
#include <memory>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;

// Speed constants (in mph or your chosen units)
const float SPEED_MAX     = 3.0f;  // Full speed
const float SPEED_CAUTION = 2.0f;  // Reduced speed
const float SPEED_STOP    = 0.0f;  // Stop

// Finite State Machine (FSM) 
enum class RobotState {
  NORMAL,   // Full speed
  CAUTION,  // Reduced speed
  STOP      // Full stop
};

class StateMachine {
public:
  StateMachine() : current_state_(RobotState::NORMAL) {}

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
    // A timer to periodically update the FSM using live sensor data (every 500ms)
    timer_ = this->create_wall_timer(500ms, std::bind(&WheelchairNode::updateFSM, this));
    RCLCPP_INFO(this->get_logger(), "Wheelchair node started.");
  }

  // Call this after the node is fully constructed
  void initialize() {
    sensor_subscriber_ = std::make_shared<SensorsSubscriber>(this->shared_from_this());
  }

private:
  // This method polls the latest sensor data, computes a decision, and updates the FSM.
  void updateFSM() {
    // Retrieve the latest sensor data.
    // It is assumed that SensorData has a field named ultrasonic_front_0 (in meters)
    auto sensor_data = sensor_subscriber_->get_latest_sensor_data();

    // Use the ultrasonic sensor reading for our decision.
    float ultrasonic_distance = sensor_data.ultrasonic_front_0;

    // Determine the speed limit based on thresholds:
    // - If distance >= 1.0 m, then SPEED_MAX (NORMAL)
    // - If between 0.5 and 1.0 m, then SPEED_CAUTION (CAUTION)
    // - If less than 0.5 m, then SPEED_STOP (STOP)
    float speed_limit = SPEED_MAX;
    if (ultrasonic_distance >= 1.0f) {
      speed_limit = SPEED_MAX;
    } else if (ultrasonic_distance >= 0.5f) {
      speed_limit = SPEED_CAUTION;
    } else {
      speed_limit = SPEED_STOP;
    }

    // Update the FSM with the computed speed limit
    fsm_.update(speed_limit);

    // Log the sensor reading, computed speed limit, and current FSM state.
    RCLCPP_INFO(this->get_logger(),
                "Ultrasonic front: %.2f m, Computed Speed: %.2f, FSM State: %s",
                ultrasonic_distance, speed_limit, fsm_.getStateName().c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<SensorsSubscriber> sensor_subscriber_;
  StateMachine fsm_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  // Create the node using std::make_shared to allow shared_from_this()
  auto node = std::make_shared<WheelchairNode>();
  
  // Initialize the sensor subscriber now that the node is fully constructed
  node->initialize();

  // Spin to process callbacks 
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
