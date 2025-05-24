/* main.cpp – left-pivot obstacle avoidance
   drive_mode: 0=manual, 1=joystick pass-through, 2=autonomous */

#include "main.h"
#include "rclcpp/rclcpp.hpp"
#include "sensors_subscriber.hpp"
#include "obstacle_subscriber.hpp"
#include "ref_speed_publisher.hpp"
#include <std_msgs/msg/int32.hpp>
#include <atomic>
#include <thread>

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
    const double DT = 1.0 / 30.0;

    while (rclcpp::ok())
    {
        RefSpeed cmd{SPEED, SPEED};

        /* ─ mode 1: joystick passthrough ─ */
        if (drive_mode.load() == 0) {
            auto s = sensors_sub->get_latest_sensor_data();
            cmd.leftSpeed  = s.left_speed;
            cmd.rightSpeed = s.right_speed;
            ref_pub->trigger_publish(cmd);
            loop.sleep();
            continue;
        }

        /* ─ mode 2: autonomous ─ */
        if (drive_mode.load() == 2) {
            bool front   =  front_clear.load();
            bool left_ok =  left_turn_clear.load();

            switch (mode)
            {
            case Mode::STRAIGHT:
                if (!front) {
                    if (!left_ok) {
                        mode = Mode::STOP;           // nowhere to go
                    } else {
                        pivot_dir = -1;              // always left
                        timer     = 0;
                        mode      = Mode::PIVOT;
                    }
                }
                break;

            case Mode::PIVOT:
                if (!left_ok) { mode = Mode::STOP; break; }
                timer += DT;
                cmd.leftSpeed  = (pivot_dir == +1) ? SPEED       : INNER_SPEED;
                cmd.rightSpeed = (pivot_dir == +1) ? INNER_SPEED : SPEED;
                if (timer >= PIVOT_DURATION) {
                    pivot_dir = -pivot_dir;          // swing back right
                    timer     = 0;
                    mode      = Mode::RETURN_PIVOT;
                }
                break;

            case Mode::RETURN_PIVOT:
                if (!left_ok) { mode = Mode::STOP; break; }
                timer += DT;
                cmd.leftSpeed  = (pivot_dir == +1) ? SPEED       : INNER_SPEED;
                cmd.rightSpeed = (pivot_dir == +1) ? INNER_SPEED : SPEED;
                if (timer >= RETURN_PIVOT_DURATION) {
                    timer = 0;
                    mode  = front ? Mode::STRAIGHT : Mode::STOP;
                }
                break;

            case Mode::STOP:
                cmd.leftSpeed = cmd.rightSpeed = 0;
                if (front) mode = Mode::STRAIGHT;
                break;
            }

            ref_pub->trigger_publish(cmd);
            loop.sleep();
            continue;
        }

        /* mode 0: manual/off */
        loop.sleep();
    }

    exec.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}

