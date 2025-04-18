cmake_minimum_required(VERSION 3.8)
project(wheelchair_code_module)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wheelchair_sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)  
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rmw REQUIRED)
find_package(wheelchair_sensor_msgs REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters visualization)
find_package(rcl_interfaces REQUIRED)

# Add your executable
add_executable(wheelchair
  src/main.cpp
  src/fan_publisher.cpp
  src/light_publisher.cpp
  src/ref_speed_publisher.cpp
  src/fingerprint_subscriber.cpp
  src/sensors_subscriber.cpp
  src/obstacle_publisher.cpp
)

# Declare ROS dependencies
ament_target_dependencies(
  wheelchair
  rclcpp
  wheelchair_sensor_msgs
  sensor_msgs
  pcl_conversions
  std_msgs
)

# Include directories (Eigen + your headers)
target_include_directories(
  wheelchair
  PRIVATE
  ${EIGEN3_INCLUDE_DIRS}                     # <-- Eigen
  ${CMAKE_CURRENT_SOURCE_DIR}/headers
  ${CMAKE_CURRENT_SOURCE_DIR}/headers/wheelchair_code_module
  ${PCL_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}
  ${rosidl_typesupport_interface_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
)

# Link against Eigen
target_link_libraries(
  wheelchair
  Eigen3::Eigen
)

# Install
install(TARGETS
  wheelchair
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
