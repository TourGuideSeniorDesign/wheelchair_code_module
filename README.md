# Wheelchair_code_module

This ROS2 package contains the code that runs on the Jetson Nano Orin on the BAD. Its primary purpose is LiDAR data processing and navigation. It also includes a module for controlling the fan speed based on the Jetson's temperature.

Here is the full list of included files:
- Obstacle Publisher Node - Subscribes to the Mid-360 LiDAR data and processes it
- Main - Subscribes to all the sensor data and makes navigation decisions
- Stopping - An earlier prototype of using LiDAR detection to interupt navigation
- Gyro - A prototype of using the IMU on the BAD to turn a number of degrees
- Temp Monitor - Monitors the temperature of the Jetson Nano and runs the fans accordingly

## Dependencies

- ROS2 Humble
- [wheelchair_sensor_msgs](https://github.com/WheelchairSeniorDesign/wheelchair_sensor_msgs)
- Eigen
- A Mid-360 LiDAR sensor for LiDAR data

## How to install

- Clone this repo into your `ros2_ws/src` folder (or other ROS2 workspace folder)
- Run the command `colcon build` in the `ros2_ws` folder (see the ROS2 documentation for more details

## How to run
Run the command corresponding to the program that you want to run:
- Obstacle Publisher Node - `ros2 run wheelchair_code_module obstacle_publisher_node`
- Main - `ros2 run wheelchair_code_module wheelchair`
- Stopping - `ros2 run wheelchair_code_module stopping`
- Gyro - `ros2 run wheelchair_code_module gyro`
- Temp Monitor - `ros2 run wheelchair_code_module temp_monitor`
