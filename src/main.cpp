/* main.cpp – left-pivot obstacle avoidance
   drive_mode: 0 = manual / joystick pass-through
               1 = joystick + safety
               2 = autonomous                                          */

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
std::atomic_bool left_turn_clear{true};   //  we only pivot left
std::atomic_bool right_turn_clear{true};  //  kept for wiring
std::atomic_int  drive_mode{0};

/* ── FSM for autonomous mode ────────────────────────────── */
enum class Mode { STRAIGHT, PIVOT, RETURN_PIVOT, STOP };
Mode   mode      = Mode::STRAIGHT;
double timer     = 0.0;
int    pivot_dir = -1;   // −1 = left   (right never used)

/* ───────────────────────────────────────────────────────── */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("wheelchair_code_module");

    /* pubs / subs */
    auto sensors_sub  = std::make_shared<SensorsSubscriber>(node);
    auto obstacle_sub = std::make_shared<ObstacleSubscriber>(
        node, front_clear, back_clear, left_turn_clear, right_turn_clear);
    auto ref_pub      = std::make_shared<RefSpeedPublisher>(node);

    /* receive drive-mode commands from Electron GUI */
    auto electron_sub = node->create_subscription<std_msgs::msg::Int32>(
        "electron_selfdrive", 10,
        [node](std_msgs::msg::Int32::SharedPtr m)
        {
            drive_mode.store(m->data, std::memory_order_relaxed);
            RCLCPP_INFO(node->get_logger(), "[RX] drive-mode=%d", m->data);
        });

    /* spin ROS callbacks in a background thread */
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread spin_thread([&]{ exec.spin(); });

    /* main control loop – 30 Hz */
    rclcpp::Rate loop(30);
    const double DT = 1.0 / 30.0;

    while (rclcpp::ok())
    {
        /* Default command: straight at SPEED  */
        RefSpeed cmd{SPEED, SPEED};

        /* ─────────── MODE 0 : manual / pure joystick ─────────── */
        if (drive_mode.load() == 0)
        {
            auto s = sensors_sub->get_latest_sensor_data();
            cmd.leftSpeed  = s.left_speed;
            cmd.rightSpeed = s.right_speed;

            ref_pub->trigger_publish(cmd);
            loop.sleep();
            continue;                            // skip rest of loop
        }

        /* ─────────── MODE 1 : joystick + obstacle safety ─────── */
        else if (drive_mode.load() == 1)
        {
            /*  grab latest joystick input  */
            auto s = sensors_sub->get_latest_sensor_data();
            cmd.leftSpeed  = s.left_speed;
            cmd.rightSpeed = s.right_speed;

            bool front_ok = front_clear.load();   // true  = path clear
            bool back_ok  = back_clear.load();

            RefSpeed ref_speed;

            if (!front_ok && !back_ok)            // both directions blocked
            {
                ref_speed = {0, 0};
            }
            else
            {
                /* block forward if front is NOT clear */
                if (!front_ok)
                {
                    if (cmd.leftSpeed  > 0) cmd.leftSpeed  = 0;
                    if (cmd.rightSpeed > 0) cmd.rightSpeed = 0;
                }

                /* block reverse if back is NOT clear */
                if (!back_ok)
                {
                    if (cmd.leftSpeed  < 0) cmd.leftSpeed  = 0;
                    if (cmd.rightSpeed < 0) cmd.rightSpeed = 0;
                }

                ref_speed = cmd;                 // safe command
            }

            ref_pub->trigger_publish(ref_speed);
            loop.sleep();
            continue;
        }

        /* ─────────── MODE 2 : autonomous left-pivot logic ────── */
        else if (drive_mode.load() == 2)
        {
            bool front_ok =  front_clear.load();
            bool left_ok  =  left_turn_clear.load();

            switch (mode)
            {
            /* driving straight until obstacle */
            case Mode::STRAIGHT:
                if (!front_ok)                   // front blocked
                {
                    if (!left_ok)
                    {   mode = Mode::STOP; }     // nowhere to go
                    else
                    {   pivot_dir = -1;          // always pivot left
                        timer     = 0.0;
                        mode      = Mode::PIVOT;
                    }
                }
                break;

            /* pivot in place to the left */
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

            /* pivot back slightly to realign */
            case Mode::RETURN_PIVOT:
                if (!left_ok) { mode = Mode::STOP; break; }
                timer += DT;
                cmd.leftSpeed  = (pivot_dir == +1) ? SPEED       : INNER_SPEED;
                cmd.rightSpeed = (pivot_dir == +1) ? INNER_SPEED : SPEED;
                if (timer >= RETURN_PIVOT_DURATION) {
                    timer = 0;
                    mode  = front_ok ? Mode::STRAIGHT : Mode::STOP;
                }
                break;

            /* emergency stop */
           case Mode::STOP:
                cmd.leftSpeed = cmd.rightSpeed = 0;
                if (front_ok) mode = Mode::STRAIGHT;
                break;
            }

            ref_pub->trigger_publish(cmd);
            loop.sleep();
            continue;
        }
        
        loop.sleep();
    }

    /* clean shutdown */
    exec.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
