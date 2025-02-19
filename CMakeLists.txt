cmake_minimum_required(VERSION 3.8)
project(wheelchair_code_module)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Specify include directories for fan_speed target
include_directories(headers headers/wheelchair_code_module)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wheelchair_sensor_msgs REQUIRED)

add_executable(wheelchair
        src/main.cpp
        src/fan_publisher.cpp
        src/light_publisher.cpp
        src/ref_speed_publisher.cpp
        src/fingerprint_subscriber.cpp
        src/sensors_subscriber.cpp)
ament_target_dependencies(wheelchair rclcpp wheelchair_sensor_msgs)

install(TARGETS
        wheelchair
        DESTINATION lib/${PROJECT_NAME})



ament_package()
