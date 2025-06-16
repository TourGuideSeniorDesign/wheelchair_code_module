/* main.cpp – left-pivot obstacle avoidance
   drive_mode: 0=manual, 1=joystick pass-through, 2=autonomous */

#include "main.h"
#include "rclcpp/rclcpp.hpp"
#include "sensors_subscriber.hpp"
#include "obstacle_subscriber.hpp"
#include "ref_speed_publisher.hpp"
#include "heading.hpp"
#include <std_msgs/msg/int32.hpp>
#include <atomic>
#include <thread>
#include <chrono>

/* ── config ─────────────────────────────────────────────── */
constexpr int    SPEED                 = 15;
constexpr int    INNER_SPEED           = SPEED / 4;    // 3
constexpr double PIVOT_DURATION        = 2.5;          // s
constexpr double RETURN_PIVOT_DURATION = 2.5;          // s

/* ── shared flags ───────────────────────────────────────── */
std::atomic_bool front_clear{true};
std::atomic_bool back_clear{true};
std::atomic_bool left_turn_clear{true};   //  ← we *only* use this one
std::atomic_bool right_turn_clear{true};  //  kept for wiring; ignored
std::atomic_int  drive_mode{0};

/* ── FSM ────────────────────────────────────────────────── */
enum class Mode { STRAIGHT, PIVOT, RETURN_PIVOT, STOP };
Mode   mode      = Mode::STRAIGHT;
double timer     = 0.0;
int    pivot_dir = -1;   // −1 = left  (right never used)

/* ───────────────────────────────────────────────────────── */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("wheelchair_code_module");

    auto sensors_sub  = std::make_shared<SensorsSubscriber>(node);
    auto obstacle_sub = std::make_shared<ObstacleSubscriber>(
        node, front_clear, back_clear, left_turn_clear, right_turn_clear);
    auto ref_pub      = std::make_shared<RefSpeedPublisher>(node);

    auto electron_sub = node->create_subscription<std_msgs::msg::Int32>(
        "electron_selfdrive", 10,
        [node](std_msgs::msg::Int32::SharedPtr m){
            drive_mode.store(m->data, std::memory_order_relaxed);
            RCLCPP_INFO(node->get_logger(), "[RX] drive-mode=%d", m->data);
        });

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread spin_thread([&]{ exec.spin(); });

    rclcpp::Rate loop(30);

    while (rclcpp::ok())
    {
        RefSpeed cmd{SPEED, SPEED};

        /* ─ mode 0: joystick passthrough ─ */
        if (drive_mode.load() == 0) {
            auto s = sensors_sub->get_latest_sensor_data();
            cmd.leftSpeed  = s.left_speed;
            cmd.rightSpeed = s.right_speed;

            //temp code for the heading
            //auto heading_value = heading(s.magnetic_field_x, s.magnetic_field_y);
            //RCLCPP_INFO(node->get_logger(), "Heading: %.2f degrees", heading_value);

            ref_pub->trigger_publish(cmd);
            loop.sleep();
            continue;
        }
        /* ─ mode 2: autonomous ─ */
        if (drive_mode.load() == 2) {

            const float target_angle = 90.0;
            static float curr_angle = 0;
            static float prev_error = 0.0f;
            static auto prevTime = std::chrono::steady_clock::now(); // initilization

            auto sensorValue = sensors_sub->get_latest_sensor_data();
            float gyro_vel_z = sensorValue.angular_velocity_z * (180.0f / M_PI);
            auto currentTime = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = currentTime - prevTime;
            double dt = elapsed.count();  // seconds

            curr_angle += gyro_vel_z * dt;

            //PD controller
            float error = target_angle - curr_angle;
            float derivative = (error - prev_error) / dt;

            const float Kp = 1.0f;
            const float Kd = 0.2f;

            float output = Kp * error + Kd * derivative;

            const float maxSpeed = SPEED;
            output = std::clamp(output, -maxSpeed, maxSpeed);

            RefSpeed cmd{0, static_cast<int8_t>(output)};
            ref_pub->trigger_publish(cmd);

            RCLCPP_INFO(node->get_logger(), "Angle = %.2f deg, Output speed = %.2f", curr_angle, output);

            prev_error = error;

            // if(abs(curr_angle) < target_angle){
            //     RefSpeed cmd{0, output};
            //     ref_pub->trigger_publish(cmd);
            //     RCLCPP_INFO(node->get_logger(), "Turning… angle = %.3f deg / %.3f deg", curr_angle, target_angle);
            // } else {
            //     RefSpeed cmd{0, 0};
            //     ref_pub->trigger_publish(cmd);
            //     RCLCPP_INFO(node->get_logger(), "Reached target angle");
            // }

            prevTime = currentTime;

            loop.sleep();
            continue;
        }

        /* mode extra*/
        loop.sleep();
    }

    exec.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
